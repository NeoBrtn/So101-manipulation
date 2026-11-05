"""Forward / inverse kinematics of the SO-101 arm with Pinocchio.

The model is built from the so101_description URDF, with the gripper joint locked:
the IK only sees the 5 arm joints. The TCP is ``gripper_frame_link``, between the
fingertips. In that frame, z is the approach axis (out of the jaws) and x is the
jaw closing direction.

Inverse kinematics: closed-loop IK (CLIK), iterated damped least squares on

    e = [p_target - p, rotation error]      (world-aligned axes, at the TCP)

The rotation error, and its weight matrix ``W``, depend on what is asked:

- ``closing_dir`` given: full orientation, ``R log3(R^T R_target)``, ``W = I``.
- approach axis only: ``z x z_target``, ``W = I - z z^T``. The rotation *about*
  the approach axis (yaw) gets zero weight: that is how a 5-DOF arm handles a 6-DOF
  target.

Two ways to combine position and orientation:

- strict (``ik``): one weighted task,  ``dq = J_w^T (J_w J_w^T + l^2 I)^-1 e_w``.
  Succeeds only if both are reached.
- prioritized (``ik_reach``): position first, orientation in its null space,

      dq1 = J_p^+ e_p,   dq = dq1 + (J_r N)^+ (e_r - J_r dq1),   N = I - J_p^+ J_p

  The position is always reached when it can be, with the orientation as close as
  the remaining joints allow (e.g. "as vertical as possible").

A solve tries a few seeds, starting from the caller's (the current or previous
waypoint, so consecutive solutions stay close), and returns the first one within
tolerance, or the best found. Each seed stops early when the error stalls, which
keeps unreachable targets cheap to reject.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

ARM_JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TCP_FRAME = "gripper_frame_link"
DOWN = np.array([0.0, 0.0, -1.0])

# Solver settings
MAX_ITERATIONS = 200
DAMPING = 1e-3             # lambda: keeps the step bounded near singularities
SECONDARY_DAMPING = 0.05   # prioritized mode: the orientation is often unreachable, keep it tame
MAX_STEP = 0.25            # rad, per joint and per iteration
ROTATION_WEIGHT = 0.3      # m per rad (strict mode): 1 rad of orientation error ~ 30 cm
POSITION_TOLERANCE = 2e-4  # m, convergence of the inner loop
ANGLE_TOLERANCE = 2e-3     # rad
STALL_WINDOW = 15          # iterations: give up when the error...
STALL_RATIO = 0.95         # ...decreased by less than 5 % over the window

# Accepted by callers by default (an IK result is "reachable" below these)
REACH_POSITION_TOLERANCE = 0.005   # m
REACH_ANGLE_TOLERANCE_DEG = 3.0
# A grasp tilt is accepted only when reached this precisely (else: tilt more)
GRASP_POSITION_TOLERANCE = 0.001   # m
GRASP_ANGLE_TOLERANCE_DEG = 1.0


def default_urdf_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"


@dataclass
class IkResult:
    q: np.ndarray                # 5 arm joints (rad)
    position_error: float        # m
    angle_error_deg: float       # approach axis (or full orientation) error

    def ok(self, position_tol=REACH_POSITION_TOLERANCE, angle_tol_deg=REACH_ANGLE_TOLERANCE_DEG) -> bool:
        return self.position_error <= position_tol and self.angle_error_deg <= angle_tol_deg


def grasp_rotation(approach, closing_dir) -> np.ndarray:
    """TCP orientation with z along `approach` and the jaws closing along `closing_dir`
    (2D horizontal or 3D; its component along the approach is removed)."""
    z = np.asarray(approach, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.zeros(3)
    x[:len(closing_dir)] = closing_dir
    x -= (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def tilted_approach(position, tilt) -> np.ndarray:
    """Approach direction tilted by `tilt` rad from vertical, leaning away from the base."""
    radial = np.array([position[0], position[1], 0.0])
    radial /= max(np.linalg.norm(radial), 1e-9)
    return np.sin(tilt) * radial + np.cos(tilt) * DOWN


def _dls(J, e, damping=DAMPING):
    """Damped least-squares solution of J dq = e."""
    return J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(J.shape[0]), e)


def _clamp(dq):
    return dq * min(1.0, MAX_STEP / max(np.abs(dq).max(), 1e-12))


class ArmKinematics:
    """FK / IK of the 5 arm joints. Joint vectors are ordered as ARM_JOINT_NAMES."""

    def __init__(self, urdf_path: str | Path | None = None, tcp_frame: str = TCP_FRAME):
        full = pin.buildModelFromUrdf(str(urdf_path or default_urdf_path()))
        locked = [full.getJointId(n) for n in full.names[1:] if n not in ARM_JOINT_NAMES]
        self.model = pin.buildReducedModel(full, locked, pin.neutral(full))
        self.data = self.model.createData()
        if [self.model.names[i] for i in range(1, self.model.njoints)] != list(ARM_JOINT_NAMES):
            raise ValueError(f"unexpected joints in the URDF: {list(self.model.names)}")
        self._tcp = self.model.getFrameId(tcp_frame)
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        # shoulder_pan turns the arm toward -y for positive angles on the SO-101: measure it
        self._pan_sign = float(np.sign(self.fk(np.array([0.3, 0, 0, 0, 0]))[1, 3]))

    # -- forward kinematics -------------------------------------------------------------

    def fk(self, q) -> np.ndarray:
        """4x4 pose of the TCP in base_link."""
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        return self.data.oMf[self._tcp].homogeneous.copy()

    def tcp_position(self, q) -> np.ndarray:
        return self.fk(q)[:3, 3]

    def pan_towards(self, xy) -> float:
        """shoulder_pan angle that roughly points the arm at the horizontal position xy
        (the pan axis is offset from the base origin: a seed, not an exact solution)."""
        return self._pan_sign * float(np.arctan2(xy[1], xy[0]))

    # -- inverse kinematics -------------------------------------------------------------

    def ik(self, position, approach=DOWN, closing_dir=None, q_seed=None, prioritized=False) -> IkResult:
        """Joints putting the TCP at ``position``.

        approach:     desired TCP z-axis (world), or None for position only.
        closing_dir:  desired jaw closing direction (TCP x-axis). None leaves the
                      rotation about the approach axis free.
        q_seed:       initial guess (e.g. the current joints).
        prioritized:  position first, orientation as close as possible (see module doc).
        """
        position = np.asarray(position, dtype=float)
        R_target = None
        if closing_dir is not None:
            R_target = grasp_rotation(DOWN if approach is None else approach, closing_dir)
            approach = R_target[:, 2]
        elif approach is not None:
            approach = np.asarray(approach, dtype=float)
            approach = approach / np.linalg.norm(approach)

        best = None
        for seed in self._seeds(position, q_seed):
            result = self._solve(position, approach, R_target, seed, prioritized)
            converged = result.position_error <= POSITION_TOLERANCE * 5
            if converged and (prioritized or result.angle_error_deg <= np.degrees(ANGLE_TOLERANCE) * 5):
                return result
            if best is None or _cost(result) < _cost(best):
                best = result
        return best

    def ik_top_down(self, position, q_seed=None, closing_dir=None) -> IkResult:
        """Gripper pointing straight down (yaw free unless closing_dir is given)."""
        return self.ik(position, DOWN, closing_dir, q_seed)

    def ik_grasp(self, position, q_seed=None, closing_dir=None, max_tilt_deg=30.0,
                 tilt_step_deg=5.0) -> tuple[IkResult, np.ndarray]:
        """Top-down if possible, else the smallest outward tilt (in tilt_step_deg steps)
        that reaches `position` with the jaws along closing_dir. Returns (result, approach)."""
        best = None
        for tilt in np.radians(np.arange(0.0, max_tilt_deg + 1e-9, tilt_step_deg)):
            approach = tilted_approach(position, tilt)
            result = self.ik(position, approach, closing_dir, q_seed)
            if result.ok(GRASP_POSITION_TOLERANCE, GRASP_ANGLE_TOLERANCE_DEG):
                return result, approach
            if best is None or _cost(result) < _cost(best[0]):
                best = (result, approach)
        return best

    def ik_reach(self, position, q_seed=None, approach=DOWN, closing_dir=None) -> IkResult:
        """Reaches the position, with the orientation as close to `approach` (and
        closing_dir) as the arm allows. For waypoints where the exact orientation does
        not matter, e.g. lifting, or above the drop box."""
        return self.ik(position, approach, closing_dir, q_seed, prioritized=True)

    def cartesian_path(self, q_start, end_position, steps, approach=DOWN, closing_dir=None) -> list[IkResult]:
        """Straight-line TCP path from FK(q_start) to end_position at a fixed orientation,
        each point warm-started from the previous one (e.g. the descent onto an object)."""
        start = self.tcp_position(q_start)
        path, q = [], np.asarray(q_start, dtype=float)
        for s in np.linspace(0.0, 1.0, steps + 1)[1:]:
            result = self.ik(start + s * (np.asarray(end_position) - start), approach, closing_dir, q)
            path.append(result)
            q = result.q
        return path

    # -- internals ----------------------------------------------------------------------

    def _seeds(self, position, q_seed):
        pan = self.pan_towards(position)
        seeds = []
        if q_seed is not None:
            q_seed = np.asarray(q_seed, dtype=float)
            seeds.append(q_seed)
            turned = q_seed.copy()
            turned[0] = pan      # a local solver struggles to swing the base far
            seeds.append(turned)
        roll = 0.0 if q_seed is None else q_seed[4]
        seeds += [np.array([pan, lift, elbow, wrist, roll])
                  for lift in (-0.5, 0.3) for elbow in (-0.5, 0.5) for wrist in (0.5, 1.2)]
        return [np.clip(s, self.lower, self.upper) for s in seeds]

    def _solve(self, position, approach, R_target, q, prioritized) -> IkResult:
        model, data = self.model, self.data
        q = q.copy()
        window_error = np.inf
        for i in range(MAX_ITERATIONS):
            pin.framesForwardKinematics(model, data, q)
            pose = data.oMf[self._tcp]
            e_pos = position - pose.translation
            e_rot, W_rot = self._rotation_error(pose.rotation, approach, R_target)
            if np.linalg.norm(e_pos) < POSITION_TOLERANCE and np.linalg.norm(e_rot) < ANGLE_TOLERANCE:
                break
            # Stalled (local minimum, or a joint limit in the way): stop early
            if i % STALL_WINDOW == 0:
                error = float(np.linalg.norm(e_pos) + ROTATION_WEIGHT * np.linalg.norm(e_rot))
                if error > STALL_RATIO * window_error:
                    break
                window_error = error

            J = pin.computeFrameJacobian(model, data, q, self._tcp, pin.LOCAL_WORLD_ALIGNED)
            dq = self._step(J, W_rot, e_pos, e_rot, prioritized)
            # A joint at its limit and pushed further would swallow part of every step:
            # take it out of the problem for this iteration and solve with the others.
            blocked = ((q <= self.lower + 1e-6) & (dq < 0)) | ((q >= self.upper - 1e-6) & (dq > 0))
            if blocked.any():
                J[:, blocked] = 0.0
                dq = self._step(J, W_rot, e_pos, e_rot, prioritized)
            q = np.clip(q + _clamp(dq), self.lower, self.upper)
        return self._result(q, position, approach, R_target)

    @staticmethod
    def _step(J, W_rot, e_pos, e_rot, prioritized):
        J_p, J_r = J[:3], W_rot @ J[3:]
        if not prioritized:
            return _dls(np.vstack([J_p, ROTATION_WEIGHT * J_r]), np.concatenate([e_pos, ROTATION_WEIGHT * e_rot]))
        dq = _clamp(_dls(J_p, e_pos))
        N = np.eye(J.shape[1]) - np.linalg.pinv(J_p, rcond=1e-4) @ J_p
        return dq + N @ _clamp(_dls(J_r @ N, e_rot - J_r @ dq, SECONDARY_DAMPING))

    @staticmethod
    def _rotation_error(R, approach, R_target):
        """(error vector in world axes, weight matrix of the rotation rows)."""
        if R_target is not None:
            return R @ pin.log3(R.T @ R_target), np.eye(3)
        if approach is None:
            return np.zeros(3), np.zeros((3, 3))
        z = R[:, 2]
        # rotation about the approach axis is free: project it out
        return np.cross(z, approach), np.eye(3) - np.outer(z, z)

    def _result(self, q, position, approach, R_target) -> IkResult:
        T = self.fk(q)
        err = float(np.linalg.norm(T[:3, 3] - position))
        if R_target is not None:
            ang = float(np.linalg.norm(pin.log3(T[:3, :3].T @ R_target)))
        elif approach is not None:
            ang = float(np.arccos(np.clip(T[:3, 2] @ approach, -1.0, 1.0)))
        else:
            ang = 0.0
        return IkResult(q, err, float(np.degrees(ang)))


def _cost(result: IkResult) -> float:
    return result.position_error + 0.002 * result.angle_error_deg   # 1 deg ~ 2 mm
