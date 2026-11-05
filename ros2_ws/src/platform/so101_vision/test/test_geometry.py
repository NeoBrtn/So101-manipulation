import numpy as np

from so101_vision.geometry import pixel_to_plane, project, quaternion_to_matrix

K = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])


def looking_down(height):
    """Camera above the origin, optical axis pointing straight down."""
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])   # x right, y down in the image = -y world
    T[:3, 3] = [0, 0, height]
    return T


def test_center_pixel_hits_below_camera():
    p = pixel_to_plane(K, looking_down(0.5), (160, 120), 0.025)
    assert np.allclose(p, [0, 0, 0.025])


def test_project_round_trip():
    T = looking_down(0.6)
    T[:3, 3] = [0.3, -0.1, 0.6]
    for point in ([0.25, 0.05, 0.0], [0.4, -0.2, 0.05]):
        uv = project(K, T, point)
        assert np.allclose(pixel_to_plane(K, T, uv, point[2]), point)


def test_plane_behind_camera():
    assert pixel_to_plane(K, looking_down(0.5), (160, 120), 1.0) is None


def test_quaternion_identity_and_yaw():
    assert np.allclose(quaternion_to_matrix(0, 0, 0, 1), np.eye(3))
    s = np.sin(np.pi / 4)
    R = quaternion_to_matrix(0, 0, s, s)   # 90 deg about z
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])
