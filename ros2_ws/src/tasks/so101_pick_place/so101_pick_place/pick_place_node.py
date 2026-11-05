#!/usr/bin/env python3
"""pick_place_node.py - phase 1: pick the cube, drop it in the box.

Inputs:  /joint_states, /object_position, /drop_box_position (perception_node)
Output:  /joint_command only (never talks to the backend)
         ~/state, ~/result (std_msgs/String)
Service: /pick_and_place (std_srvs/Trigger) starts one cycle; continuous:=true loops.

State machine (50 Hz tick, never blocks: perception is read as "last known value"):

    INIT -> IDLE -(trigger)-> WAIT_FOR_CUBE -> CHECK_REACH -(unreachable)-> WAIT_FOR_CUBE
                                                  | plan ok
                       CHECK_REACH <-(moved)- APPROACH -> VERIFY -(lost)-> HOME
                                                              | still there
                                      GRASP -> LIFT -> PLACE -> HOME -> CONFIRM
    CONFIRM: cube in the box -> done; else retry (WAIT_FOR_CUBE) up to max_retries.
"""
import time
from collections import deque

import numpy as np
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_manipulation.arm_interface import ArmInterface, spin_node
from so101_manipulation.kinematics import ArmKinematics
from so101_manipulation.trajectory import GRIPPER_CLOSED, GRIPPER_OPEN, Waypoint
from so101_pick_place.planner import REST_POSE, PlannerConfig, Unreachable, in_box, plan_pick_place

CONTROL_HZ = 50.0
GRIPPER_TIME = 1.2         # s to open / close
STABLE_SAMPLES = 5         # recent detections that must agree before planning
STABLE_SPREAD = 0.005      # m
JOINT_STATES_TIMEOUT = 0.5  # s: driver gone


class PickPlaceNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("pick_place")
        p = self.declare_parameter
        self._continuous = p("continuous", False).value
        self._home_on_start = p("home_on_start", True).value
        self._move_threshold = p("move_threshold", 0.015).value
        self._stale_timeout = p("stale_timeout", 1.0).value
        self._wait_timeout = p("wait_timeout", 10.0).value
        self._verify_timeout = p("verify_timeout", 1.5).value
        self._grasp_if_unseen = p("grasp_if_unseen", True).value
        self._confirm_delay = p("confirm_delay", 1.5).value
        self._confirm_timeout = p("confirm_timeout", 5.0).value
        self._box_half_size = p("box_half_size", 0.05).value
        self._max_retries = p("max_retries", 3).value
        d = PlannerConfig()
        self._cfg = PlannerConfig(
            grasp_height=p("grasp_height", d.grasp_height).value,
            pregrasp_height=p("pregrasp_height", d.pregrasp_height).value,
            lift_height=p("lift_height", d.lift_height).value,
            drop_height=p("drop_height", d.drop_height).value,
            min_reach=p("min_reach", d.min_reach).value,
            max_reach=p("max_reach", d.max_reach).value,
        )

        self._kin = ArmKinematics()
        self._arm = ArmInterface(self)
        self._cube_history = deque(maxlen=STABLE_SAMPLES)   # (position, receive time)
        self._box = None
        self._target = None        # cube position the current plan is based on
        self._rejected = None      # last unreachable cube position (not re-reported)
        self._plan = None
        self._retries = 0
        self._requested = False

        self._state = None
        self._t_state = 0.0
        self._pub_state = self.create_publisher(String, "~/state", 10)
        self._pub_result = self.create_publisher(String, "~/result", 10)
        self.create_subscription(PoseStamped, "/object_position", self._cb_cube, 10)
        self.create_subscription(PoseStamped, "/drop_box_position", self._cb_box, 10)
        self.create_service(Trigger, "/pick_and_place", self._srv_start)
        self.create_timer(1.0 / CONTROL_HZ, self._step)
        self._goto("INIT")
        self.get_logger().info(f"Pick & place ready ({'continuous' if self._continuous else 'on /pick_and_place'}).")

    # -- inputs -------------------------------------------------------------------------

    def _cb_cube(self, msg: PoseStamped):
        p = msg.pose.position
        self._cube_history.append((np.array([p.x, p.y, p.z]), time.monotonic()))

    def _cb_box(self, msg: PoseStamped):
        p = msg.pose.position
        self._box = np.array([p.x, p.y, p.z])

    def _srv_start(self, request, response):
        if self._state != "IDLE":
            response.success, response.message = False, f"busy ({self._state})"
            return response
        self._requested = True
        response.success, response.message = True, "pick & place started"
        return response

    def _latest_cube(self, since=None):
        """Last detection if fresh (and received after `since`), else None."""
        if not self._cube_history:
            return None
        pos, t = self._cube_history[-1]
        now = time.monotonic()
        if now - t > self._stale_timeout or (since is not None and t < since):
            return None
        return pos

    def _stable_cube(self):
        now = time.monotonic()
        recent = [p for p, t in self._cube_history if now - t < self._stale_timeout]
        if len(recent) < STABLE_SAMPLES or np.ptp(recent, axis=0).max() > STABLE_SPREAD:
            return None
        return np.mean(recent, axis=0)

    def _moved(self, cube) -> bool:
        return cube is not None and np.linalg.norm(cube[:2] - self._target[:2]) > self._move_threshold

    # -- state machine ------------------------------------------------------------------

    def _goto(self, state, waypoints=None):
        if self._state is not None and state != self._state:
            self.get_logger().info(f"{self._state} -> {state}")
        self._state = state
        self._t_state = time.monotonic()
        if waypoints is not None:
            self._arm.move(waypoints)
        self._pub_state.publish(String(data=state))

    def _elapsed(self):
        return time.monotonic() - self._t_state

    def _finish(self, success: bool, message: str):
        """End of a cycle: report, then wait for the next cube (continuous) or trigger."""
        self._pub_result.publish(String(data=("SUCCESS: " if success else "FAILURE: ") + message))
        if success:
            self.get_logger().info(f"Pick & place SUCCESS: {message}")
        else:
            self.get_logger().warn(f"Pick & place FAILURE: {message}")
        self._retries = 0
        self._goto("WAIT_FOR_CUBE" if self._continuous else "IDLE")

    def _rest(self, state, gripper_after=GRIPPER_CLOSED):
        self._goto(state, [Waypoint("rest", REST_POSE, self._arm.streamer.gripper_cmd),
                           Waypoint("close", REST_POSE, gripper_after, GRIPPER_TIME)])

    def _step(self):
        if not self._arm.ready:
            return
        if self._state == "IDLE":
            # The driver holds the last target; other nodes may move the arm meanwhile.
            if (self._requested or self._continuous) and self._arm.state_age() < JOINT_STATES_TIMEOUT:
                self._requested = False
                self._arm.hold_current(self._arm.gripper_pct_hint())
                self._goto("WAIT_FOR_CUBE")
            return
        if self._state == "INIT" and self._arm.streamer.q_cmd is None:
            self._arm.hold_current(self._arm.gripper_pct_hint())
            if self._home_on_start:
                self._rest("INIT")
            else:
                self._goto("IDLE")
                return
        if self._arm.state_age() > JOINT_STATES_TIMEOUT:
            self._arm.streamer.cancel()
            self._requested = False
            self._finish(False, f"no /joint_states for {JOINT_STATES_TIMEOUT} s: is the driver running?")
            self._goto("IDLE")
            return

        done = self._arm.stream()
        getattr(self, f"_on_{self._state.lower()}")(done)

    def _on_init(self, done):
        if done:
            self._goto("WAIT_FOR_CUBE" if self._continuous else "IDLE")

    def _on_wait_for_cube(self, done):
        cube = self._stable_cube()
        if cube is None or self._box is None:
            if not self._continuous and self._elapsed() > self._wait_timeout:
                what = "cube" if self._box is not None else "drop box"
                self._finish(False, f"no stable {what} detection (out of the camera view? perception down?)")
            return
        if in_box(cube, self._box, self._box_half_size):
            if not self._continuous:
                self._finish(True, "the cube is already in the box")
            return
        if self._rejected is not None and np.linalg.norm(cube[:2] - self._rejected[:2]) < self._move_threshold:
            return   # same unreachable cube as before: already reported
        self._target = cube
        self._goto("CHECK_REACH")

    def _on_check_reach(self, done):
        t0 = time.monotonic()
        try:
            self._plan = plan_pick_place(self._kin, self._target, self._box, self._cfg)
        except Unreachable as e:
            self._rejected = self._target
            self._finish(False, f"cube out of reach at {np.round(self._target, 3).tolist()}: {e}")
            return
        self._rejected = None
        errors = ", ".join(f"{k} {v:.1f}" for k, v in self._plan.errors_mm.items())
        self.get_logger().info(f"Cube at {np.round(self._target, 3).tolist()}: planned in "
                               f"{(time.monotonic() - t0) * 1000:.0f} ms, grasp tilt "
                               f"{self._plan.grasp_tilt_deg:.0f} deg, IK errors (mm): {errors}")
        self._goto("APPROACH", [Waypoint("pre-grasp", self._plan.pregrasp, GRIPPER_OPEN)])

    def _replan_or_abort(self, cube):
        self._arm.streamer.cancel()
        self._retries += 1
        moved = np.linalg.norm(cube[:2] - self._target[:2]) * 1000
        if self._retries > self._max_retries:
            self.get_logger().warn(f"Cube moved ({moved:.0f} mm) too many times, giving up.")
            self._rest("HOME_ABORT")
            return
        self.get_logger().info(f"Cube moved by {moved:.0f} mm: replanning (retry {self._retries}).")
        self._target = cube
        self._goto("CHECK_REACH")

    def _on_approach(self, done):
        cube = self._latest_cube()
        if self._moved(cube):
            self._replan_or_abort(cube)
        elif done:
            self._goto("VERIFY")

    def _on_verify(self, done):
        # Last look before the gripper hides the cube.
        cube = self._latest_cube(since=self._t_state)
        if cube is not None:
            if self._moved(cube):
                self._replan_or_abort(cube)
            else:
                self._grasp()
        elif self._elapsed() > self._verify_timeout:
            if self._grasp_if_unseen:
                # From above, the gripper often hides the cube: trust the last estimate.
                self._grasp()
            else:
                self._rest("HOME_ABORT")
                self.get_logger().warn("Cube lost at pre-grasp.")

    def _grasp(self):
        descent = [Waypoint(f"descend {i + 1}/{len(self._plan.descent)}", q, GRIPPER_OPEN)
                   for i, q in enumerate(self._plan.descent)]
        self._goto("GRASP", descent + [Waypoint("close", self._plan.grasp, GRIPPER_CLOSED, GRIPPER_TIME)])

    def _on_grasp(self, done):
        if done:
            self._goto("LIFT", [Waypoint("lift", q, GRIPPER_CLOSED) for q in self._plan.lift])

    def _on_lift(self, done):
        if done:
            self._goto("PLACE", [Waypoint("above box", self._plan.drop, GRIPPER_CLOSED),
                                 Waypoint("open", self._plan.drop, GRIPPER_OPEN, GRIPPER_TIME)])

    def _on_place(self, done):
        if done:
            self._rest("HOME")

    def _on_home(self, done):
        if done:
            self._goto("CONFIRM")

    def _on_home_abort(self, done):
        if done:
            self._finish(False, "aborted (cube lost or moved too often)")

    def _on_confirm(self, done):
        # Wait for perception to see the final scene (the arm is out of the way now).
        if self._elapsed() < self._confirm_delay:
            return
        cube = self._latest_cube(since=self._t_state + self._confirm_delay)
        if cube is None:
            if self._elapsed() > self._confirm_timeout:
                self._finish(False, "cube not seen after the drop")
            return
        if in_box(cube, self._box, self._box_half_size):
            self._finish(True, f"cube in the box (retries: {self._retries})")
        elif self._retries < self._max_retries:
            self._retries += 1
            self.get_logger().warn(f"Cube not in the box ({np.round(cube, 3).tolist()}): retry {self._retries}.")
            self._cube_history.clear()
            self._goto("WAIT_FOR_CUBE")
        else:
            self._finish(False, f"cube not in the box after {self._max_retries} retries")


def main(args=None):
    spin_node(PickPlaceNode, args)


if __name__ == "__main__":
    main()
