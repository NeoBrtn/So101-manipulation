"""Pinhole camera geometry (pure numpy).

Monocular camera, no depth: a pixel only gives a ray. Its 3D point comes from an
assumption, here that it lies on a known horizontal plane z = z_plane (e.g. an object
standing on the table):

    d_cam = K^-1 [u, v, 1]        ray in the optical frame (x right, y down, z forward)
    d     = R_wc d_cam            same ray in the world frame, from the camera pose
    P     = O + t d,  t = (z_plane - O_z) / d_z
"""
import numpy as np


def quaternion_to_matrix(x, y, z, w) -> np.ndarray:
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pixel_ray(K, T_wc, uv):
    """(origin, unit direction) in the world of the ray through pixel uv."""
    d = T_wc[:3, :3] @ (np.linalg.solve(K, np.array([uv[0], uv[1], 1.0])))
    return T_wc[:3, 3].copy(), d / np.linalg.norm(d)


def intersect_plane(origin, direction, z_plane):
    """Point where the ray meets z = z_plane, or None (parallel, or behind the camera)."""
    if abs(direction[2]) < 1e-9:
        return None
    t = (z_plane - origin[2]) / direction[2]
    if t <= 0:
        return None
    return origin + t * direction


def pixel_to_plane(K, T_wc, uv, z_plane):
    origin, direction = pixel_ray(K, T_wc, uv)
    return intersect_plane(origin, direction, z_plane)


def project(K, T_wc, point):
    """World point -> pixel (u, v), or None if behind the camera."""
    T_cw = np.linalg.inv(T_wc)
    p = T_cw[:3, :3] @ np.asarray(point, dtype=float) + T_cw[:3, 3]
    if p[2] <= 0:
        return None
    uv = K @ (p / p[2])
    return uv[:2]
