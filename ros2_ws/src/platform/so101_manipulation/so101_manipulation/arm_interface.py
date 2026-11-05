"""ROS side of the arm, shared by every task node.

    ArmInterface(node)  subscribes /joint_states, publishes /joint_command, and owns a
                        TrajectoryStreamer. The task calls `stream()` on its control
                        tick; nothing here blocks.

Task nodes never talk to the backend (sim or real): /joint_command is the only way to
move the arm (radians for the 5 arm joints, 0-100 % for the gripper).
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from sensor_msgs.msg import JointState

from so101_manipulation.kinematics import ARM_JOINT_NAMES
from so101_manipulation.trajectory import GripperMonitor, TrajectoryStreamer, Waypoint

JOINT_NAMES = list(ARM_JOINT_NAMES) + ["gripper"]
# Gripper joint limits in the URDF (rad): 0 % = closed (lower), 100 % = open (upper)
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)


class ArmInterface:
    def __init__(self, node, streamer: TrajectoryStreamer | None = None):
        self._node = node
        self.streamer = streamer or TrajectoryStreamer()
        self.q = None                 # measured arm joints (rad)
        self.gripper = None           # measured gripper (rad, URDF joint)
        self.last_state_t = 0.0       # monotonic time of the last /joint_states
        self._gripper_monitor = GripperMonitor()
        self._pub = node.create_publisher(JointState, "/joint_command", 10)
        # Own callback group: with spin_node()'s two threads, the joint states keep
        # flowing while the task computes (e.g. plans with the IK)
        node.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())

    @property
    def ready(self) -> bool:
        return self.q is not None

    def state_age(self) -> float:
        return time.monotonic() - self.last_state_t

    def gripper_pct_hint(self) -> float:
        """Measured gripper opening in % (to hold it where it is)."""
        if self.gripper is None:
            return self.streamer.gripper_cmd
        lo, hi = GRIPPER_RANGE_RAD
        return float(np.clip((self.gripper - lo) / (hi - lo) * 100.0, 0.0, 100.0))

    def _cb_joint_states(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        now = time.monotonic()
        if all(n in pos for n in ARM_JOINT_NAMES):
            self.q = np.array([pos[n] for n in ARM_JOINT_NAMES])
            self.last_state_t = now
        if "gripper" in pos:
            self.gripper = pos["gripper"]
            self._gripper_monitor.update(pos["gripper"], now)

    # -- motion -------------------------------------------------------------------------

    def hold_current(self, gripper=None):
        """Re-anchor the streamer on the measured joints (start of a task, after idling)."""
        self.streamer.reset(self.q, gripper)

    def move(self, waypoints: list[Waypoint]):
        self.streamer.start(waypoints)

    def stream(self) -> bool:
        """Advances the current motion and publishes the command. True when done."""
        done = self.streamer.step(time.monotonic(), self.q, self._gripper_monitor.last_moving_t)
        self.send(self.streamer.q_cmd, self.streamer.gripper_cmd)
        return done

    def send(self, q, gripper_pct=None):
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(v) for v in q]
        if gripper_pct is not None:
            msg.name.append("gripper")
            msg.position.append(float(gripper_pct))
        self._pub.publish(msg)


def spin_node(node_class, args=None):
    """rclpy boilerplate of the task nodes: init, spin on two threads, clean shutdown."""
    rclpy.init(args=args)
    node = node_class()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
