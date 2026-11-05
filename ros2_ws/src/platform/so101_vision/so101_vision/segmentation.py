"""Colour segmentation (OpenCV HSV: H in 0-180)."""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Blob:
    center: np.ndarray    # (u, v) pixel centroid
    area: int             # px
    bbox: tuple           # (u, v, width, height)

    def touches_border(self, image_shape, margin=1) -> bool:
        """Cut by the image edge: the object is partly out of view and its centroid is
        biased toward the inside of the image."""
        u, v, w, h = self.bbox
        height, width = image_shape[:2]
        return u <= margin or v <= margin or u + w >= width - margin or v + h >= height - margin


def hsv_mask(hsv, ranges):
    """Union of the HSV ranges [((h, s, v) low, (h, s, v) high), ...], opened against noise.
    A red needs two ranges: its hue wraps around 0/180."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def largest_blob(mask, min_area=15):
    """Largest connected component of the mask, or None."""
    n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[best, cv2.CC_STAT_AREA])
    if area < min_area:
        return None
    bbox = tuple(int(x) for x in stats[best, :4])   # left, top, width, height
    return Blob(np.array(centroids[best]), area, bbox)
