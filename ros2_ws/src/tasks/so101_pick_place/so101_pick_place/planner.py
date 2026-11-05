"""Pick & place motion plan (pure Python, no ROS).

All waypoints are solved once, before the arm moves: once the gripper is above the
cube it hides it from the camera, so the plan relies on the last good detection.

    grasp        top-down at grasp_height; tilted outward (up to max_grasp_tilt) where
                 a vertical gripper cannot reach (far end of the workspace)
    pre-grasp    back along the approach axis, pregrasp_height - grasp_height away
    descent      straight line along the approach axis to the grasp (Cartesian path)
    lift         back up, then to lift_height
    drop         above the box centre, at drop_height

Grasp orientation: the jaws close perpendicular to the base -> cube direction (least
wrist roll). The moving jaw swings open on one side: it is turned away from the box,
otherwise it can hit the box wall or shove the cube when the cube lies next to the box
(1/12 successful grasps vs 12/12 in simulation tests).

A top-down gripper is only reachable up to z ~ 9 cm (wrist_flex limit): the lift and
drop waypoints, where the orientation does not matter, use prioritized IK (position
exact, orientation as close as possible).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from so101_manipulation.kinematics import DOWN, ArmKinematics

# Folded rest pose, turned toward -y, out of the camera's view of the cube area
# (folded at pan 0, the arm hides up to half of a nearby cube).
REST_POSE = np.array([1.85, -1.7, 1.55, 0.7, 0.0])


@dataclass
class PlannerConfig:
    grasp_height: float = 0.02       # m, TCP; the cube centre is at 0.025
    pregrasp_height: float = 0.08    # above the cube top (0.05)
    lift_height: float = 0.11
    drop_height: float = 0.11        # the cube bottom clears the box rim (0.05)
    min_reach: float = 0.12          # m, horizontal distance base -> cube
    max_reach: float = 0.34
    max_grasp_tilt_deg: float = 30.0  # outward tilt allowed when top-down is out of reach
    ik_tolerance: float = 0.01       # m
    descent_steps: int = 4


@dataclass
class PickPlan:
    grasp_tilt_deg: float
    pregrasp: np.ndarray
    descent: list[np.ndarray]        # ends at the grasp pose
    lift: list[np.ndarray]
    drop: np.ndarray
    errors_mm: dict = field(default_factory=dict)

    @property
    def grasp(self) -> np.ndarray:
        return self.descent[-1]


class Unreachable(Exception):
    pass


def closing_direction(cube_xy, box_xy) -> np.ndarray:
    radial = np.arctan2(cube_xy[1], cube_xy[0])
    d = np.array([-np.sin(radial), np.cos(radial)])
    return d if d @ (np.asarray(box_xy) - np.asarray(cube_xy)) >= 0 else -d


def check_reach(cube, cfg: PlannerConfig):
    """Cheap test before any IK: horizontal distance within [min_reach, max_reach]."""
    r = float(np.hypot(cube[0], cube[1]))
    if not cfg.min_reach <= r <= cfg.max_reach:
        raise Unreachable(f"{r * 100:.1f} cm from the base, outside [{cfg.min_reach * 100:.0f}, "
                          f"{cfg.max_reach * 100:.0f}] cm")


def plan_pick_place(kin: ArmKinematics, cube, box, cfg: PlannerConfig | None = None) -> PickPlan:
    """Joint waypoints to pick the cube at `cube` (centre, m) and drop it in `box`.
    Raises Unreachable."""
    cfg = cfg or PlannerConfig()
    cube, box = np.asarray(cube, dtype=float), np.asarray(box, dtype=float)
    check_reach(cube, cfg)
    closing = closing_direction(cube[:2], box[:2])
    x, y = cube[:2]
    errors = {}

    def reach(position, seed, name, approach=DOWN, closing_dir=None):
        """Position first, orientation as close as possible (prioritized IK)."""
        result = kin.ik_reach(position, seed, approach, closing_dir)
        errors[name] = result.position_error * 1000
        return result

    grasp_pos = np.array([x, y, cfg.grasp_height])
    grasp, approach = kin.ik_grasp(grasp_pos, REST_POSE, closing, cfg.max_grasp_tilt_deg)
    errors["grasp"] = grasp.position_error * 1000
    if not grasp.ok(cfg.ik_tolerance):
        raise Unreachable(f"no grasp within {cfg.max_grasp_tilt_deg:.0f} deg of vertical "
                          f"({errors['grasp']:.0f} mm off)")
    tilt = float(np.degrees(np.arccos(np.clip(-approach[2], -1.0, 1.0))))

    # pre-grasp back along the approach axis (straight above for a top-down grasp)
    pre_pos = grasp_pos - approach * (cfg.pregrasp_height - cfg.grasp_height) / max(-approach[2], 1e-6)
    pre_aligned = kin.ik(pre_pos, approach, closing, grasp.q)
    if pre_aligned.ok(cfg.ik_tolerance):
        pre = pre_aligned
        errors["pre-grasp"] = pre.position_error * 1000
    else:
        pre = reach(pre_pos, grasp.q, "pre-grasp", approach, closing)
    # Keep the grasp orientation while carrying the cube as far as the arm allows
    lift = reach([x, y, cfg.lift_height], pre.q, "lift", approach, closing)
    drop = reach([box[0], box[1], cfg.drop_height], lift.q, "drop")

    bad = [k for k, v in errors.items() if v > cfg.ik_tolerance * 1000]
    if bad:
        raise Unreachable("IK failed for " + ", ".join(f"{k} ({errors[k]:.0f} mm)" for k in bad))

    # Straight descent along the approach axis when the pre-grasp is aligned with it
    descent = [grasp.q]
    if pre_aligned.ok(cfg.ik_tolerance):
        path = kin.cartesian_path(pre.q, grasp_pos, cfg.descent_steps, approach, closing)
        if all(p.ok(cfg.ik_tolerance) for p in path):
            descent = [p.q for p in path]
    return PickPlan(tilt, pre.q, descent, [pre.q, lift.q], drop.q, errors)


def in_box(cube, box, half_size=0.05) -> bool:
    return bool(np.all(np.abs(np.asarray(cube)[:2] - np.asarray(box)[:2]) < half_size))
