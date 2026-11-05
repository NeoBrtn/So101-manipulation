#!/usr/bin/env python3
"""motion_server_node.py - task-independent arm utilities.

    /end_effector_pose     geometry_msgs/PoseStamped, FK of /joint_states (10 Hz)
    /go_to_target          so101_interfaces/GoToTarget: move the TCP to an XYZ
                           (Pinocchio IK, gripper as vertical as possible)
    /set_joint_positions   so101_interfaces/SetJointPositions: joint-space move
                           (degrees for the arm, 0-100 % for the gripper)

The services return as soon as the motion starts; the reached position and its error
(FK of the measured joints) are logged when it ends. The node only publishes on
/joint_command while one of its motions runs, so it can live next to a task node.

    ros2 service call /go_to_target so101_interfaces/srv/GoToTarget \
        "{target_pose: {pose: {position: {x: 0.25, y: 0.0, z: 0.05}}}}"
"""
import numpy as np
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped

from so101_interfaces.srv import GoToTarget, SetJointPositions
from so101_manipulation.arm_interface import ArmInterface, spin_node
from so101_manipulation.kinematics import ArmKinematics
from so101_manipulation.trajectory import Waypoint

CONTROL_HZ = 50.0
POSE_HZ = 10.0


def quaternion_from_matrix(R):
    """(x, y, z, w) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return float(x), float(y), float(z), float(w)


class MotionServer(rclpy.node.Node):
    def __init__(self):
        super().__init__("motion_server")
        self._kin = ArmKinematics()
        self._arm = ArmInterface(self)
        self._active = False
        self._target = None          # XYZ of the current /go_to_target, for the error log

        self._pub_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self.create_service(GoToTarget, "/go_to_target", self._srv_go_to_target)
        self.create_service(SetJointPositions, "/set_joint_positions", self._srv_set_joints)
        self.create_timer(1.0 / CONTROL_HZ, self._step)
        self.create_timer(1.0 / POSE_HZ, self._publish_pose)
        self.get_logger().info("Motion server ready.")

    def _srv_go_to_target(self, request, response):
        if not self._arm.ready:
            response.success, response.message = False, "no /joint_states yet"
            return response
        p = request.target_pose.pose.position
        target = np.array([p.x, p.y, p.z])
        result = self._kin.ik_reach(target, self._arm.q)
        if not result.ok(angle_tol_deg=180.0):
            response.success = False
            response.message = f"unreachable: best IK solution {result.position_error * 1000:.1f} mm away"
            return response
        self._start(Waypoint("go_to_target", result.q, self._arm.gripper_pct_hint()), target)
        response.success = True
        response.message = (f"moving, IK error {result.position_error * 1000:.2f} mm, "
                            f"approach {result.angle_error_deg:.0f} deg from vertical")
        return response

    def _srv_set_joints(self, request, response):
        if not self._arm.ready:
            response.success, response.message = False, "no /joint_states yet"
            return response
        if len(request.positions) != 6:
            response.success, response.message = False, "expected 6 values (5 joints in deg + gripper %)"
            return response
        q = np.clip(np.radians(request.positions[:5]), self._kin.lower, self._kin.upper)
        self._start(Waypoint("set_joint_positions", q, float(request.positions[5])), None)
        response.success, response.message = True, "moving"
        return response

    def _start(self, waypoint, target):
        if not self._active:
            self._arm.hold_current(self._arm.gripper_pct_hint())
        self._arm.move([waypoint])
        self._target = target
        self._active = True

    def _step(self):
        if not self._active:
            return
        if self._arm.stream():
            self._active = False
            if self._target is not None:
                reached = self._kin.tcp_position(self._arm.q)
                self.get_logger().info(
                    f"Reached {np.round(reached, 4).tolist()}, error "
                    f"{np.linalg.norm(reached - self._target) * 1000:.1f} mm (FK of the measured joints)")

    def _publish_pose(self):
        if not self._arm.ready:
            return
        T = self._kin.fk(self._arm.q)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = quaternion_from_matrix(T[:3, :3])
        self._pub_pose.publish(msg)


def main(args=None):
    spin_node(MotionServer, args)


if __name__ == "__main__":
    main()
