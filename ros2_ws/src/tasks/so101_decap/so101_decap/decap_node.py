#!/usr/bin/env python3
"""decap_node.py - phase 2: find the bottle on the table, put it in the fixture, unscrew
its cap, drop the cap in the cap box, then put the bottle in the bottle box.

The fixture is passive (ribs between the bottle's feet), so there is no lock / unlock
step. Design: docs/phase2_decap.md.

    ros2 service call /decap std_srvs/srv/Trigger
    ros2 topic echo /decap/result

Inputs: /joint_states, /bottle_position (perception_node). Output: /joint_command only.

State machine (50 Hz, plan first, then stream; never blocks on perception):

    IDLE -(/decap)-> DETECT_BOTTLE -> PLAN -> APPROACH_BOTTLE -(moved)-> PLAN
         -> GRASP_BOTTLE -> LIFT_BOTTLE -> TRANSPORT -> INSERT_1 .. INSERT_n -> RELEASE
         -> APPROACH_CAP -> GRASP_CAP -> UNSCREW_1 .. UNSCREW_n -> LIFT_CAP -> PLACE_CAP
         -> APPROACH_NECK -> GRASP_NECK -> REMOVE_BOTTLE -> PLACE_BOTTLE -> HOME -> IDLE
    any failure (not seen, unreachable, nothing in the gripper, cannot seat) -> RECOVER

Insertion: the bottle's feet must slot between the fixture's ribs, and its yaw is
unknown. The bottle is pushed down slightly below its seated height; if the arm stays
higher (FK of the measured joints: the feet rest on the rib tops), the bottle is turned
about its axis and pushed again, until it drops in (search over the 72 deg pattern).

Every grasp is checked from the gripper position: closed on something, the jaws stop
before fully closed; closed on nothing, the part was missed or slipped.
"""
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_decap.planner import REST_POSE, DecapConfig, DecapPlanner, Insert, Unreachable
from so101_manipulation.arm_interface import ArmInterface, spin_node
from so101_manipulation.kinematics import ArmKinematics
from so101_manipulation.trajectory import GRIPPER_CLOSED, GRIPPER_OPEN, Waypoint

CONTROL_HZ = 50.0
GRIPPER_TIME = 1.2          # s to open / close
JOINT_STATES_TIMEOUT = 0.5
STABLE_SAMPLES = 5          # recent detections that must agree before planning
STABLE_SPREAD = 0.005       # m


@dataclass
class Step:
    state: str
    waypoints: list
    check_holding: bool = False   # after the motion: the gripper must hold something
    insert: Insert | None = None  # after the motion: is the bottle seated?
    watch_bottle: bool = False    # during the motion: replan if the bottle moved


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
            carry_bottom=p("carry_bottom", d.carry_bottom).value,
            insert_preload=p("insert_preload", d.insert_preload).value,
            min_reach=p("min_reach", d.min_reach).value,
            max_reach=p("max_reach", d.max_reach).value,
        )
        self._stroke_time = p("stroke_time", 6.0).value           # s per unscrewing stroke
        self._holding_min_pct = p("holding_min_pct", 3.0).value   # below: the jaws closed on nothing
        self._seat_tolerance = p("seat_tolerance", 0.004).value   # m above the seated height: not in
        self._move_threshold = p("move_threshold", 0.015).value   # m: the bottle moved -> replan
        self._stale_timeout = p("stale_timeout", 1.0).value
        self._detect_timeout = p("detect_timeout", 10.0).value
        self._max_retries = p("max_retries", 3).value
        self._home_on_start = p("home_on_start", True).value

        self._planner = DecapPlanner(ArmKinematics(), self._cfg)
        self._arm = ArmInterface(self)
        self._bottle = deque(maxlen=STABLE_SAMPLES)   # (xy, receive time)
        self._target = None                           # bottle xy the pick plan is based on
        self._seated_z = 0.0                          # TCP height once the bottle is seated
        self._steps: list[Step] = []
        self._step = None
        self._requested = False
        self._retries = 0
        self._failure = ""
        self._state = None
        self._t_state = time.monotonic()
        self._pub_state = self.create_publisher(String, "~/state", 10)
        self._pub_result = self.create_publisher(String, "~/result", 10)
        self.create_subscription(PoseStamped, "/bottle_position", self._cb_bottle, 10)
        self.create_service(Trigger, "/decap", self._srv_decap)
        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self._set_state("INIT")
        self.get_logger().info("Decap ready: ros2 service call /decap std_srvs/srv/Trigger")

    # -- inputs -------------------------------------------------------------------------

    def _cb_bottle(self, msg: PoseStamped):
        p = msg.pose.position
        self._bottle.append((np.array([p.x, p.y]), time.monotonic()))

    def _srv_decap(self, request, response):
        if self._state != "IDLE":
            response.success, response.message = False, f"busy ({self._state})"
            return response
        self._requested = True
        response.success, response.message = True, "decapping started"
        return response

    def _stable_bottle(self):
        now = time.monotonic()
        recent = [xy for xy, t in self._bottle if now - t < self._stale_timeout]
        if len(recent) < STABLE_SAMPLES or np.ptp(recent, axis=0).max() > STABLE_SPREAD:
            return None
        return np.mean(recent, axis=0)

    def _latest_bottle(self):
        if not self._bottle:
            return None
        xy, t = self._bottle[-1]
        return xy if time.monotonic() - t < self._stale_timeout else None

    # -- sequencing ---------------------------------------------------------------------

    def _set_state(self, state):
        if self._state is not None and state != self._state:
            self.get_logger().info(f"{self._state} -> {state}")
        self._state = state
        self._t_state = time.monotonic()
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

    def _pick_steps(self, pick) -> list[Step]:
        closed, open_ = GRIPPER_CLOSED, GRIPPER_OPEN
        steps = [
            Step("APPROACH_BOTTLE", [Waypoint("above the bottle", pick.neck_pregrasp, open_),
                                     Waypoint("around the neck", pick.neck_grasp, open_)], watch_bottle=True),
            Step("GRASP_BOTTLE", [Waypoint("close", pick.neck_grasp, closed, GRIPPER_TIME)], check_holding=True),
            Step("LIFT_BOTTLE", [Waypoint("lift", pick.lift, closed)], check_holding=True),
            Step("TRANSPORT", [Waypoint("above the fixture", pick.above_fixture, closed)], check_holding=True),
        ]
        for i, ins in enumerate(pick.inserts):
            steps.append(Step(f"INSERT_{i + 1}", [Waypoint(f"push at {ins.angle_deg:+.0f} deg", ins.q, closed)],
                              insert=ins))
        return steps

    def _fixture_steps(self, plan) -> list[Step]:
        closed, open_ = GRIPPER_CLOSED, GRIPPER_OPEN
        dt = self._stroke_time / max(1, len(plan.strokes[0].turn))
        steps = [
            Step("APPROACH_CAP", [Waypoint("above the cap", plan.cap_pregrasp, open_),
                                  Waypoint("around the cap", plan.cap_grasp, open_)]),
            Step("GRASP_CAP", [Waypoint("close", plan.cap_grasp, closed, GRIPPER_TIME)], check_holding=True),
        ]
        for i, stroke in enumerate(plan.strokes):
            wps = [Waypoint(f"turn {i + 1}", q, closed, dt) for q in stroke.turn]
            if i < len(plan.strokes) - 1:   # regrasp for the next stroke
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
            Step("REMOVE_BOTTLE", [Waypoint("lift", plan.bottle_lift, closed)], check_holding=True),
            Step("PLACE_BOTTLE", [Waypoint("above the bottle box", plan.bottle_above_box, closed),
                                  Waypoint("down", plan.bottle_place, closed),
                                  Waypoint("release", plan.bottle_place, open_, GRIPPER_TIME),
                                  Waypoint("retreat", plan.bottle_retreat, open_)]),
            Step("HOME", self._rest()),
        ]
        return steps

    def _plan_and_start(self, xy):
        self._set_state("PLAN")
        t0 = time.monotonic()
        try:
            pick = self._planner.plan_pick(xy)
            plan = self._planner.plan()
        except Unreachable as e:
            self._fail_cycle(f"unreachable: {e}")
            return
        self._target = xy
        self._seated_z = pick.seated_tcp_z
        self.get_logger().info(
            f"Bottle at {np.round(xy, 3).tolist()}: planned in {(time.monotonic() - t0) * 1000:.0f} ms "
            f"({len(pick.inserts)} insertion tries, {plan.info['strokes']} unscrewing strokes)")
        self._run(self._pick_steps(pick) + self._fixture_steps(plan))

    def _fail_cycle(self, why):
        self._finish(False, why)
        self._retries = 0
        self._set_state("IDLE")

    # -- tick ---------------------------------------------------------------------------

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
                self._retries = 0
                self._arm.hold_current(self._arm.gripper_pct_hint())
                self._bottle.clear()
                self._set_state("DETECT_BOTTLE")
            return
        if self._arm.state_age() > JOINT_STATES_TIMEOUT:
            self._arm.streamer.cancel()
            self._fail_cycle(f"no /joint_states for {JOINT_STATES_TIMEOUT} s: is the driver running?")
            return

        done = self._arm.stream()
        if self._state == "DETECT_BOTTLE":
            xy = self._stable_bottle()
            if xy is not None:
                self._plan_and_start(xy)
            elif time.monotonic() - self._t_state > self._detect_timeout:
                self._fail_cycle("no stable bottle detection (out of the camera view? perception down?)")
            return
        step = self._step
        if step is None:
            return
        if step.watch_bottle and not done:
            xy = self._latest_bottle()
            if xy is not None and np.linalg.norm(xy - self._target) > self._move_threshold:
                self._replan(xy)
            return
        if not done:
            return

        if step.check_holding or step.insert is not None:
            pct = self._arm.gripper_pct_hint()
            if pct < self._holding_min_pct:
                self._recover(f"{step.state}: gripper closed on nothing ({pct:.1f} %): missed or dropped the part")
                return
        if step.insert is not None:
            z = self._planner.kin.tcp_position(self._arm.q)[2]
            above = z - self._seated_z
            if above < self._seat_tolerance:
                self.get_logger().info(f"Bottle seated at {step.insert.angle_deg:+.0f} deg.")
                while self._steps and self._steps[0].insert is not None:
                    self._steps.pop(0)
                self._steps.insert(0, Step("RELEASE", [
                    Waypoint("open", step.insert.q, GRIPPER_OPEN, GRIPPER_TIME),
                    Waypoint("up", step.insert.retreat, GRIPPER_OPEN)]))
            elif not (self._steps and self._steps[0].insert is not None):
                self._recover(f"could not seat the bottle in the fixture ({above * 1000:.0f} mm too high "
                              "after the whole search)")
                return
            else:
                self.get_logger().info(f"Not seated at {step.insert.angle_deg:+.0f} deg "
                                       f"({above * 1000:.0f} mm high): turning")
        if self._steps:
            self._next_step()
            return
        # sequence over
        self._step = None
        if step.state == "RECOVER":
            self._finish(False, self._failure)
        elif step.state != "INIT":
            self._finish(True, "bottle put in the fixture and uncapped, cap in the cap box, bottle in the bottle box")
        self._set_state("IDLE")

    def _replan(self, xy):
        self._arm.streamer.cancel()
        self._retries += 1
        moved = np.linalg.norm(xy - self._target) * 1000
        if self._retries > self._max_retries:
            self._recover(f"the bottle moved ({moved:.0f} mm) too many times")
            return
        self.get_logger().info(f"Bottle moved by {moved:.0f} mm: replanning (retry {self._retries}).")
        self._bottle.clear()
        self._set_state("DETECT_BOTTLE")


def main(args=None):
    spin_node(DecapNode, args)


if __name__ == "__main__":
    main()
