#!/usr/bin/env python3
"""decap_node.py - phase 2: unscrew the cap of a bottle held in the fixture, drop the
cap in the cap box, then put the bottle in the bottle box.

v1: the bottle starts in the fixture (placed by hand); picking it up and inserting it
is v2. The fixture is passive (ribs between the bottle's feet), so there is no
lock / unlock step. Design: docs/phase2_decap.md.

    ros2 service call /decap std_srvs/srv/Trigger
    ros2 topic echo /decap/result

State machine (50 Hz, plan first, then stream; never blocks):

    IDLE -(/decap)-> PLAN -> APPROACH_CAP -> GRASP_CAP -> UNSCREW_1 .. UNSCREW_n
         -> LIFT_CAP -> PLACE_CAP -> APPROACH_NECK -> GRASP_NECK -> LIFT_BOTTLE
         -> PLACE_BOTTLE -> HOME -> IDLE
    any failure (unreachable, nothing in the gripper) -> RECOVER -> IDLE

Every grasp is checked from the gripper position: closed on something, the jaws stop
before fully closed; closed on nothing, the cap or the bottle was missed or slipped.
Unscrewing strokes rotate the gripper about the cap axis; between strokes the gripper
opens, turns back and closes again (the wrist turns ~320 deg at most).
"""
import time
from dataclasses import dataclass

import numpy as np
import rclpy
import rclpy.node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_decap.planner import REST_POSE, DecapConfig, DecapPlanner, Unreachable
from so101_manipulation.arm_interface import ArmInterface, spin_node
from so101_manipulation.kinematics import ArmKinematics
from so101_manipulation.trajectory import GRIPPER_CLOSED, GRIPPER_OPEN, Waypoint

CONTROL_HZ = 50.0
GRIPPER_TIME = 1.2          # s to open / close
JOINT_STATES_TIMEOUT = 0.5


@dataclass
class Step:
    state: str
    waypoints: list
    check_holding: bool = False   # after the motion: the gripper must hold something


class DecapNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("decap")
        p = self.declare_parameter
        d = DecapConfig()
        self._cfg = DecapConfig(
            table_z=p("table_z", d.table_z).value,
            fixture_xy=tuple(p("fixture_xy", list(d.fixture_xy)).value),
            cap_box_xy=tuple(p("cap_box_xy", list(d.cap_box_xy)).value),
            bottle_box_xy=tuple(p("bottle_box_xy", list(d.bottle_box_xy)).value),
            cap_center=p("cap_center", d.cap_center).value,
            cap_diameter=p("cap_diameter", d.cap_diameter).value,
            neck_center=p("neck_center", d.neck_center).value,
            neck_diameter=p("neck_diameter", d.neck_diameter).value,
            fixed_jaw_face=p("fixed_jaw_face", d.fixed_jaw_face).value,
            grasp_depth=p("grasp_depth", d.grasp_depth).value,
            thread_pitch=p("thread_pitch", d.thread_pitch).value,
            unscrew_turns=p("unscrew_turns", d.unscrew_turns).value,
            stroke_deg=p("stroke_deg", d.stroke_deg).value,
        )
        self._stroke_time = p("stroke_time", 6.0).value          # s per unscrewing stroke
        self._holding_min_pct = p("holding_min_pct", 3.0).value  # below: the jaws closed on nothing
        self._home_on_start = p("home_on_start", True).value

        self._planner = DecapPlanner(ArmKinematics(), self._cfg)
        self._arm = ArmInterface(self)
        self._steps: list[Step] = []
        self._step = None
        self._requested = False
        self._state = None
        self._pub_state = self.create_publisher(String, "~/state", 10)
        self._pub_result = self.create_publisher(String, "~/result", 10)
        self.create_service(Trigger, "/decap", self._srv_decap)
        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self._set_state("INIT")
        self.get_logger().info("Decap ready: ros2 service call /decap std_srvs/srv/Trigger")

    def _srv_decap(self, request, response):
        if self._state != "IDLE":
            response.success, response.message = False, f"busy ({self._state})"
            return response
        self._requested = True
        response.success, response.message = True, "decapping started"
        return response

    # -- sequencing ---------------------------------------------------------------------

    def _set_state(self, state):
        if self._state is not None and state != self._state:
            self.get_logger().info(f"{self._state} -> {state}")
        self._state = state
        self._pub_state.publish(String(data=state))

    def _run(self, steps: list[Step]):
        """Queues a sequence of steps; each one starts when the previous one is done."""
        self._steps = list(steps)
        self._next_step()

    def _next_step(self):
        if not self._steps:
            self._step = None
            return
        self._step = self._steps.pop(0)
        self._set_state(self._step.state)
        self._arm.move(self._step.waypoints)

    def _rest(self, gripper=GRIPPER_CLOSED):
        return [Waypoint("rest", REST_POSE, self._arm.streamer.gripper_cmd),
                Waypoint("close", REST_POSE, gripper, GRIPPER_TIME)]

    def _finish(self, success: bool, message: str):
        self._pub_result.publish(String(data=("SUCCESS: " if success else "FAILURE: ") + message))
        if success:
            self.get_logger().info(f"Decap SUCCESS: {message}")
        else:
            self.get_logger().warn(f"Decap FAILURE: {message}")

    def _recover(self, why: str):
        """Open, back off upward, go home, report."""
        self.get_logger().warn(f"Recovering: {why}")
        self._arm.streamer.cancel()
        self._failure = why
        q = self._arm.streamer.q_cmd
        up = self._planner.kin.ik_reach(self._planner.kin.tcp_position(q) + [0, 0, 0.05], q)
        self._run([Step("RECOVER", [Waypoint("open", q, GRIPPER_OPEN, GRIPPER_TIME),
                                    Waypoint("back off", up.q, GRIPPER_OPEN)] + self._rest())])

    # -- the sequence -------------------------------------------------------------------

    def _build_steps(self, plan) -> list[Step]:
        closed, open_ = GRIPPER_CLOSED, GRIPPER_OPEN
        dt = self._stroke_time / max(1, len(plan.strokes[0].turn))
        steps = [
            Step("APPROACH_CAP", [Waypoint("above the cap", plan.cap_pregrasp, open_),
                                  Waypoint("around the cap", plan.cap_grasp, open_)]),
            Step("GRASP_CAP", [Waypoint("close", plan.cap_grasp, closed, GRIPPER_TIME)], check_holding=True),
        ]
        for i, stroke in enumerate(plan.strokes):
            wps = [Waypoint(f"turn {i + 1}", q, closed, dt) for q in stroke.turn]
            last = i == len(plan.strokes) - 1
            if not last:   # regrasp for the next stroke
                wps += [Waypoint("open", stroke.turn[-1], open_, GRIPPER_TIME),
                        Waypoint("turn back", stroke.back, open_),
                        Waypoint("close", stroke.back, closed, GRIPPER_TIME)]
            steps.append(Step(f"UNSCREW_{i + 1}", wps, check_holding=True))
        steps += [
            Step("LIFT_CAP", [Waypoint("lift", plan.cap_lift, closed)], check_holding=True),
            Step("PLACE_CAP", [Waypoint("above the cap box", plan.cap_drop, closed),
                               Waypoint("drop", plan.cap_drop, open_, GRIPPER_TIME)]),
            Step("APPROACH_NECK", [Waypoint("above the neck", plan.neck_pregrasp, open_),
                                   Waypoint("around the neck", plan.neck_grasp, open_)]),
            Step("GRASP_NECK", [Waypoint("close", plan.neck_grasp, closed, GRIPPER_TIME)], check_holding=True),
            Step("LIFT_BOTTLE", [Waypoint("lift", plan.bottle_lift, closed)], check_holding=True),
            Step("PLACE_BOTTLE", [Waypoint("above the bottle box", plan.bottle_above_box, closed),
                                  Waypoint("down", plan.bottle_place, closed),
                                  Waypoint("release", plan.bottle_place, open_, GRIPPER_TIME),
                                  Waypoint("retreat", plan.bottle_retreat, open_)]),
            Step("HOME", self._rest()),
        ]
        return steps

    def _tick(self):
        if not self._arm.ready:
            return
        if self._state == "INIT" and self._arm.streamer.q_cmd is None:
            self._arm.hold_current(self._arm.gripper_pct_hint())
            if not self._home_on_start:
                self._set_state("IDLE")
                return
            self._run([Step("INIT", self._rest())])
        if self._state == "IDLE":
            if self._requested and self._arm.state_age() < JOINT_STATES_TIMEOUT:
                self._requested = False
                self._arm.hold_current(self._arm.gripper_pct_hint())
                self._plan_and_start()
            return
        if self._arm.state_age() > JOINT_STATES_TIMEOUT:
            self._arm.streamer.cancel()
            self._finish(False, f"no /joint_states for {JOINT_STATES_TIMEOUT} s: is the driver running?")
            self._set_state("IDLE")
            return

        done = self._arm.stream()
        if not done or self._step is None:
            return
        step = self._step
        if step.check_holding:
            pct = self._arm.gripper_pct_hint()
            if pct < self._holding_min_pct:
                what = "the cap" if "CAP" in step.state or "UNSCREW" in step.state else "the bottle"
                self._recover(f"{step.state}: gripper closed on nothing ({pct:.1f} %): missed or dropped {what}")
                return
        if self._steps:
            self._next_step()
            return
        # sequence over
        self._step = None
        if step.state == "INIT":
            pass
        elif step.state == "RECOVER":
            self._finish(False, self._failure)
        else:
            self._finish(True, "cap in the cap box, bottle in the bottle box")
        self._set_state("IDLE")

    def _plan_and_start(self):
        self._set_state("PLAN")
        t0 = time.monotonic()
        try:
            plan = self._planner.plan()
        except Unreachable as e:
            self._finish(False, f"unreachable: {e}")
            self._set_state("IDLE")
            return
        self.get_logger().info(f"Planned in {(time.monotonic() - t0) * 1000:.0f} ms: {plan.info['strokes']} strokes, "
                               f"wrist roll from {np.degrees(plan.info['wrist_roll_start']):.0f} deg")
        self._run(self._build_steps(plan))


def main(args=None):
    spin_node(DecapNode, args)


if __name__ == "__main__":
    main()
