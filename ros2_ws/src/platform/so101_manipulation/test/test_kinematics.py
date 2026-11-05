"""Pinocchio IK on the SO-101 model: round trips and the pick & place workspace."""
from pathlib import Path

import numpy as np
import pytest

from so101_manipulation.kinematics import ArmKinematics

URDF = Path(__file__).resolve().parents[2] / "so101_description" / "urdf" / "so101.urdf"
REST = np.array([1.85, -1.7, 1.55, 0.7, 0.0])


@pytest.fixture(scope="module")
def kin():
    return ArmKinematics(URDF)


def test_zero_pose_fk(kin):
    # arm stretched forward along +x, in the vertical plane of the base
    p = kin.tcp_position(np.zeros(5))
    assert p[0] > 0.3 and abs(p[1]) < 1e-3


def test_pan_points_roughly_at_target(kin):
    # a seed only: the pan axis is ~4 cm in front of the base origin
    for azimuth in np.radians([-60, -20, 0, 30, 70]):
        q = np.zeros(5)
        q[0] = kin.pan_towards([np.cos(azimuth), np.sin(azimuth)])
        p = kin.tcp_position(q)
        assert np.isclose(np.arctan2(p[1], p[0]), azimuth, atol=np.radians(8))


def test_round_trip_position_and_approach(kin):
    rng = np.random.default_rng(0)
    solved = 0
    for _ in range(40):
        q = rng.uniform(kin.lower * 0.8, kin.upper * 0.8)
        T = kin.fk(q)
        result = kin.ik(T[:3, 3], T[:3, 2], None, q_seed=q + rng.normal(0, 0.2, 5))
        solved += result.ok()
    assert solved >= 36


@pytest.mark.parametrize("radius", [0.18, 0.24])
@pytest.mark.parametrize("angle_deg", [-50, 0, 60])
def test_top_down_grasp_in_cube_area(kin, radius, angle_deg):
    a = np.radians(angle_deg)
    xy = radius * np.array([np.cos(a), np.sin(a)])
    closing = np.array([-np.sin(a), np.cos(a)])
    grasp = kin.ik_top_down([*xy, 0.02], REST, closing)
    assert grasp.ok(position_tol=0.001, angle_tol_deg=1.0)
    # the jaws close along the requested direction
    x_axis = kin.fk(grasp.q)[:3, 0]
    assert abs(x_axis[:2] @ closing) > 0.99


@pytest.mark.parametrize("angle_deg", [-50, 0, 60])
def test_far_grasp_tilts_outward(kin, angle_deg):
    a = np.radians(angle_deg)
    xy = 0.30 * np.array([np.cos(a), np.sin(a)])
    closing = np.array([-np.sin(a), np.cos(a)])
    grasp, approach = kin.ik_grasp([*xy, 0.02], REST, closing)
    assert grasp.ok()
    tilt = np.degrees(np.arccos(-approach[2]))
    assert tilt <= 30
    assert approach[:2] @ xy >= 0           # leaning away from the base
    assert abs(kin.fk(grasp.q)[:3, 0][:2] @ closing) > 0.95


def test_top_down_has_a_height_limit(kin):
    # wrist_flex saturates: a vertical gripper cannot be held above ~9 cm
    assert not kin.ik_top_down([0.24, 0.0, 0.13], REST).ok()
    # ik_reach tilts the approach instead
    assert kin.ik_reach([0.24, 0.0, 0.13], REST).ok(angle_tol_deg=90)


def test_cartesian_descent_is_straight(kin):
    start = kin.ik_top_down([0.22, 0.05, 0.08], REST).q
    path = kin.cartesian_path(start, [0.22, 0.05, 0.02], steps=5)   # top-down, yaw free
    assert all(r.ok() for r in path)
    xy = np.array([kin.tcp_position(r.q)[:2] for r in path])
    assert np.abs(xy - [0.22, 0.05]).max() < 0.002
