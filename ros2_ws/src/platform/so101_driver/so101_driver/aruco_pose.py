"""aruco_pose.py - camera pose in the world from one ArUco marker lying on the table.

Convention: the marker lies flat, its centre at (x, y, z) in the robot frame (world: x
forward from the base, y to the left, z up), the TOP of the printed marker pointing away
from the robot (+x).
"""
import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
# world <- marker: marker x (right of the printed image) = world -y,
# marker y (top of the image) = world +x, marker z (out of the paper) = world +z
WORLD_R_MARKER = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def make_detector():
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT))


def world_T_marker(xy, z=0.0):
    T = np.eye(4)
    T[:3, :3] = WORLD_R_MARKER
    T[:3, 3] = [xy[0], xy[1], z]
    return T


def camera_T_marker_candidates(detector, gray, K, D, marker_id, size):
    """Both poses of the marker in the camera optical frame that fit its 4 corners, best
    first, as [(4x4, reprojection error px), ...]; [] if the marker is not seen.

    A single flat marker seen nearly face-on is ambiguous: a second, mirrored pose fits
    almost as well (often only 1 px worse). The caller picks with some context."""
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or marker_id not in ids.flatten():
        return []
    c = corners[list(ids.flatten()).index(marker_id)].reshape(4, 2)
    s = size / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)  # IPPE_SQUARE order
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, c, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    out = []
    for rvec, tvec, err in zip(rvecs, tvecs, np.asarray(errs).flatten()):
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = np.asarray(tvec).flatten()
        out.append((T, float(err)))
    return sorted(out, key=lambda x: x[1])


def camera_T_marker(detector, gray, K, D, marker_id, size):
    """Best-fitting marker pose in the camera optical frame, or None if not seen."""
    cands = camera_T_marker_candidates(detector, gray, K, D, marker_id, size)
    return cands[0][0] if cands else None


def guess_K(width, height):
    """Uncalibrated lens: typical phone main camera (~65° horizontal field of view)."""
    f = 0.78 * width
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]])
