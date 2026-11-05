"""Occlusion-robust position estimate of a (mostly static) object.

When the arm hides part of an object, the centroid of what remains visible shifts (up
to ~15 cm for the drop box), which would make the arm chase a ghost. A detection is
therefore rejected when its blob is much smaller than usual, and a jump away from the
current estimate is only accepted once several consecutive detections agree (the
object really moved). The estimate is the median of the last accepted detections.
"""
import numpy as np


class Tracker:
    def __init__(self, min_area_ratio=0.6, max_jump=0.02, confirm_frames=5, median_window=5,
                 area_ref_decay=0.99):
        self.min_area_ratio = min_area_ratio  # blob smaller than this x the reference: hidden
        self.max_jump = max_jump              # m: farther = candidate for a new position
        self.confirm_frames = confirm_frames  # consistent frames to accept a new position
        self.median_window = median_window
        self.area_ref_decay = area_ref_decay  # per frame: the reference follows slow changes
        self.reset()

    def reset(self):
        self.estimate = None
        self._window = []
        self._ref_area = None
        self._candidates = []

    def update(self, position, area) -> bool:
        """Feeds one detection. True if accepted (the estimate is up to date)."""
        position = np.asarray(position, dtype=float)
        self._ref_area = area if self._ref_area is None else max(area, self._ref_area * self.area_ref_decay)
        if area < self.min_area_ratio * self._ref_area:
            return False

        if self.estimate is None or np.linalg.norm(position - self.estimate) < self.max_jump:
            self._candidates = []
            self._accept(position)
            return True

        if self._candidates and np.linalg.norm(position - np.mean(self._candidates, axis=0)) > self.max_jump:
            self._candidates = []
        self._candidates.append(position)
        if len(self._candidates) < self.confirm_frames:
            return False
        self._window = list(self._candidates)
        self._candidates = []
        self._accept(position)
        return True

    def _accept(self, position):
        self._window = (self._window + [position])[-self.median_window:]
        self.estimate = np.median(self._window, axis=0)
