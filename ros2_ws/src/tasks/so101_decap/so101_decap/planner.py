"""Decapping motion plan (pure Python, no ROS).

Setup (docs/phase2_decap.md): the robot stands on a riser, the bottle in the printed
fixture at a known pose. Heights below are from the table; the robot frame has the
table at z = table_z (negative: the riser).

Two facts drive the plan:

- The TCP (gripper_frame_link) sits on the fixed finger, not between the jaws: a round
  part of diameter d is held with its axis grasp_offset_x = fixed_jaw_face + d / 2 from
  the TCP, along the TCP x-axis, and grasp_depth lower than the TCP has to be (deeper in
  the jaws, otherwise only the fingertips touch and it slips).
- Unscrewing rotates the whole gripper about the CAP AXIS (not the wrist axis, which is
  7.8 mm off the TCP), rising with the thread. The wrist only turns ~320 deg, so it goes
  in strokes: turn, open, turn back, close.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from so101_manipulation.kinematics import ArmKinematics

# Folded rest pose, turned away from the work area
REST_POSE = np.array([1.85, -1.7, 1.55, 0.7, 0.0])


@dataclass
class DecapConfig:
    table_z: float = -0.18                  # m, table in the robot frame (riser height)
    fixture_xy: tuple = (0.20, 0.0)
    fixture_floor: float = 0.003            # bottle bottom above the table
    cap_box_xy: tuple = (0.12, -0.22)
    bottle_box_xy: tuple = (0.02, 0.25)
    box_floor: float = 0.003
    # bottle (heights from its bottom)
    cap_center: float = 0.2135
    cap_diameter: float = 0.028
    neck_center: float = 0.1975
    neck_diameter: float = 0.025
    # gripper
    fixed_jaw_face: float = 0.004           # m, see module doc
    grasp_depth: float = 0.008
    hover: float = 0.04                     # pre-grasp, above the grasp
    # thread
    thread_pitch: float = 0.0027            # m per turn
    unscrew_turns: float = 1.5              # turns to free the cap (1.25 + margin)
    stroke_deg: float = 240.0
    stroke_steps: int = 24
    # transport
    cap_lift: float = 0.04                  # the cap (17 mm) clears the neck
    cap_drop_height: float = 0.07           # TCP above the table when dropping the cap (box: 4 cm)
    bottle_lift: float = 0.06               # the bottle bottom must clear the fixture (38 mm)
    bottle_place_gap: float = 0.005         # bottle bottom above the box floor when released
    ik_tolerance: float = 0.003


class Unreachable(Exception):
    pass


@dataclass
class Stroke:
    turn: list[np.ndarray]                  # gripper closed, rotating about the cap axis
    back: np.ndarray                        # gripper open, rotated back (and higher by the thread)


@dataclass
class DecapPlan:
    cap_pregrasp: np.ndarray
    cap_grasp: np.ndarray
    strokes: list[Stroke]
    cap_lift: np.ndarray
    cap_drop: np.ndarray
    neck_pregrasp: np.ndarray
    neck_grasp: np.ndarray
    bottle_lift: np.ndarray
    bottle_above_box: np.ndarray
    bottle_place: np.ndarray
    bottle_retreat: np.ndarray
    info: dict = field(default_factory=dict)


def rotate_about_vertical(T, axis_xy, theta, dz=0.0):
    """Pose T rotated by theta (counter-clockwise from above) about the vertical line
    through axis_xy, then raised by dz."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    a = np.array([axis_xy[0], axis_xy[1], 0.0])
    out = T.copy()
    out[:3, 3] = R @ (T[:3, 3] - a) + a + [0.0, 0.0, dz]
    out[:3, :3] = R @ T[:3, :3]
    return out


class DecapPlanner:
    def __init__(self, kin: ArmKinematics, cfg: DecapConfig | None = None):
        self.kin = kin
        self.cfg = cfg or DecapConfig()

    def _z(self, height):
        return self.cfg.table_z + height

    def _solve(self, T, seed, name):
        r = self.kin.ik(T[:3, 3], T[:3, 2], T[:3, 0], seed)
        if not r.ok(self.cfg.ik_tolerance):
            raise Unreachable(f"{name}: IK {r.position_error * 1000:.1f} mm, {r.angle_error_deg:.1f} deg off")
        return r.q

    def _reach(self, p, seed, name, like=None):
        """Position exact; orientation as close as possible to the pose `like` (or vertical)."""
        if like is None:
            r = self.kin.ik_reach(p, seed)
        else:
            r = self.kin.ik_reach(p, seed, like[:3, 2], like[:3, 0])
        if r.position_error > self.cfg.ik_tolerance:
            raise Unreachable(f"{name}: IK {r.position_error * 1000:.1f} mm off")
        return r.q

    def held_offset(self, q, diameter):
        """Where a round part held in the gripper at joints q sits, relative to the TCP."""
        T = self.kin.fk(q)
        return -(self.cfg.fixed_jaw_face + diameter / 2) * T[:3, 0] + [0, 0, self.cfg.grasp_depth]

    def _reach_holding(self, target, diameter, seed, name, like=None):
        """Puts the held part's axis (not the TCP) at target: solve, correct by where the
        part ends up with that orientation, solve again."""
        q = self._reach(target, seed, name, like)
        for _ in range(2):
            q = self._reach(target - self.held_offset(q, diameter), q, name, like)
        return q

    def round_grasp(self, center, diameter, angle, seed=None):
        """Top-down grasp of a vertical round part, jaws closing along `angle` (rad)."""
        x = np.array([np.cos(angle), np.sin(angle), 0.0])
        tcp = np.asarray(center) + (self.cfg.fixed_jaw_face + diameter / 2) * x + [0, 0, -self.cfg.grasp_depth]
        return self.kin.ik_top_down(tcp, seed, x[:2])

    def plan(self) -> DecapPlan:
        c = self.cfg
        fx, fy = c.fixture_xy
        bottom = self._z(c.fixture_floor)
        cap = np.array([fx, fy, bottom + c.cap_center])

        # Cap grasp with the wrist near the low end of its range: a stroke turns it up
        grasp = None
        for a in np.radians(np.arange(0.0, 360.0, 15.0)):
            r = self.round_grasp(cap, c.cap_diameter, a)
            if r.ok(c.ik_tolerance) and (grasp is None or r.q[4] < grasp.q[4]):
                grasp = r
        if grasp is None:
            raise Unreachable("no top-down grasp of the cap")
        T_grasp = self.kin.fk(grasp.q)
        pre = self._solve(T_grasp @ _translate(0, 0, -c.hover), grasp.q, "cap pre-grasp")

        # Strokes
        strokes, turned, q = [], 0.0, grasp.q
        total = np.radians(c.unscrew_turns * 360.0)
        stroke = np.radians(c.stroke_deg)
        while turned < total - 1e-6:
            this = min(stroke, total - turned)
            q_start = q
            turn = []
            for i in range(1, c.stroke_steps + 1):
                th = this * i / c.stroke_steps
                T = rotate_about_vertical(T_grasp, cap[:2], th, c.thread_pitch * (turned + th) / (2 * np.pi))
                q = self._solve(T, q, f"stroke {len(strokes) + 1} at {np.degrees(th):.0f} deg")
                turn.append(q)
            turned += this
            T_back = T_grasp.copy()
            T_back[2, 3] += c.thread_pitch * turned / (2 * np.pi)
            q = self._solve(T_back, q_start, "turn back")   # seeded at the stroke start: not the short way
            strokes.append(Stroke(turn, q))

        # Straight up, same orientation: the cap slides off the neck along its axis
        T_end = self.kin.fk(strokes[-1].turn[-1])
        cap_lift = self._solve(_translate_world(T_end, 0, 0, c.cap_lift), strokes[-1].turn[-1], "cap lift")
        cap_drop = self._reach_holding(np.array([*c.cap_box_xy, self._z(c.cap_drop_height)]), c.cap_diameter,
                                       cap_lift, "cap drop")

        # Neck: any jaw direction that reaches, the closest to the cap drop pose
        neck = np.array([fx, fy, bottom + c.neck_center])
        best = None
        for a in np.radians(np.arange(0.0, 360.0, 15.0)):
            r = self.round_grasp(neck, c.neck_diameter, a, cap_drop)
            if not r.ok(c.ik_tolerance):
                continue
            if best is None or np.abs(r.q - cap_drop).max() < np.abs(best.q - cap_drop).max():
                best = r
        if best is None:
            raise Unreachable("no top-down grasp of the neck")
        neck_grasp = best.q
        T_neck = self.kin.fk(neck_grasp)
        neck_pre = self._solve(T_neck @ _translate(0, 0, -c.hover), neck_grasp, "neck pre-grasp")

        # Carry the bottle upright: same orientation, translated
        lift = _translate_world(T_neck, 0, 0, c.bottle_lift)
        bottle_lift = self._solve(lift, neck_grasp, "bottle lift")
        bx, by = c.bottle_box_xy
        place_z = self._z(c.box_floor + c.bottle_place_gap) + (T_neck[2, 3] - bottom)
        # The bottle's yaw does not matter in the box: position first, its axis on the box centre
        neck_lifted = neck + [0, 0, c.bottle_lift]
        above = self._reach_holding(np.array([bx, by, neck_lifted[2]]), c.neck_diameter, bottle_lift,
                                    "above the bottle box", T_neck)
        neck_placed = np.array([bx, by, place_z + c.grasp_depth])
        place = self._reach_holding(neck_placed, c.neck_diameter, above, "bottle place", T_neck)
        retreat = self._reach(self.kin.tcp_position(place) + [0, 0, c.hover], place, "retreat")

        return DecapPlan(pre, grasp.q, strokes, cap_lift, cap_drop, neck_pre, neck_grasp,
                         bottle_lift, above, place, retreat,
                         {"strokes": len(strokes), "wrist_roll_start": float(grasp.q[4])})


def _translate(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def _translate_world(T, x, y, z):
    out = T.copy()
    out[:3, 3] += [x, y, z]
    return out
