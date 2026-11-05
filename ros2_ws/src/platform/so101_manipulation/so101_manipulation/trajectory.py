"""Joint-space trajectory streaming (pure Python, no ROS).

The driver takes position targets on /joint_command and the servos track them. To get
smooth, predictable motions, tasks do not send the final target at once: a queue of
waypoints is interpolated (smoothstep: no velocity jump at either end) and streamed
at the control rate. A waypoint is done once its duration elapsed AND the measured
joints settled on it (or a timeout passed), and, for gripper moves, once the jaws
stopped moving: closed on an object, they stop before the commanded value.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

GRIPPER_OPEN = 100.0      # % (LeRobot convention)
GRIPPER_CLOSED = 0.0


@dataclass
class Waypoint:
    name: str
    q: np.ndarray                  # 5 arm joints (rad)
    gripper: float                 # 0-100 %
    duration: float | None = None  # s; None = from the joint distance and the speed limit

    def __post_init__(self):
        self.q = np.asarray(self.q, dtype=float)


class GripperMonitor:
    """Tells when the gripper last moved, from its measured position."""

    def __init__(self, stall_speed=0.05):
        self._stall_speed = stall_speed   # rad/s
        self._last = None
        self.last_moving_t = 0.0

    def update(self, position: float, now: float):
        if self._last is not None and now > self._last[1]:
            speed = abs(position - self._last[0]) / (now - self._last[1])
            if speed > self._stall_speed:
                self.last_moving_t = now
        self._last = (position, now)


class TrajectoryStreamer:
    """Streams a queue of waypoints as interpolated joint commands."""

    def __init__(self, joint_speed=0.8, min_duration=0.8, settle_tolerance=0.03,
                 settle_timeout=2.0, gripper_stall_time=0.3):
        self.joint_speed = joint_speed                  # rad/s of the fastest joint
        self.min_duration = min_duration                # s
        self.settle_tolerance = settle_tolerance        # rad, every arm joint
        self.settle_timeout = settle_timeout            # s after the nominal duration
        self.gripper_stall_time = gripper_stall_time    # s without gripper motion
        self.q_cmd = None
        self.gripper_cmd = GRIPPER_CLOSED
        self._queue: list[Waypoint] = []
        self._current: Waypoint | None = None
        self._start_q = None
        self._start_gripper = None
        self._t0 = 0.0
        self._duration = 0.0

    # -- control ------------------------------------------------------------------------

    def reset(self, q, gripper=None):
        """Start streaming from q (e.g. the measured joints), dropping any motion."""
        self.q_cmd = np.asarray(q, dtype=float).copy()
        if gripper is not None:
            self.gripper_cmd = float(gripper)
        self.cancel()

    def start(self, waypoints):
        """Replaces the queue. Interpolation starts from the last command."""
        self._queue = list(waypoints)
        self._current = None

    def cancel(self):
        """Stops at the current command (the arm holds where it is commanded)."""
        self._queue = []
        self._current = None

    @property
    def busy(self) -> bool:
        return self._current is not None or bool(self._queue)

    @property
    def current(self) -> str | None:
        return self._current.name if self._current else None

    def step(self, now: float, q_meas, gripper_last_moving_t: float = 0.0) -> bool:
        """Advances the interpolation. Returns True when the queue is done."""
        if self._current is None:
            if not self._queue:
                return True
            self._begin(self._queue.pop(0), now)
        wp = self._current
        elapsed = now - self._t0
        s = min(1.0, elapsed / self._duration)
        s = 3 * s**2 - 2 * s**3   # smoothstep
        self.q_cmd = self._start_q + (wp.q - self._start_q) * s
        self.gripper_cmd = self._start_gripper + (wp.gripper - self._start_gripper) * s

        settled = q_meas is not None and np.max(np.abs(np.asarray(q_meas) - wp.q)) < self.settle_tolerance
        if wp.gripper != self._start_gripper:
            settled = settled and now - gripper_last_moving_t > self.gripper_stall_time
        if elapsed >= self._duration and (settled or elapsed > self._duration + self.settle_timeout):
            self._current = None
            return not self._queue
        return False

    def _begin(self, wp: Waypoint, now: float):
        self._current = wp
        self._start_q = self.q_cmd.copy()
        self._start_gripper = self.gripper_cmd
        self._t0 = now
        if wp.duration is not None:
            self._duration = wp.duration
        else:
            self._duration = max(self.min_duration, float(np.max(np.abs(wp.q - self._start_q))) / self.joint_speed)
