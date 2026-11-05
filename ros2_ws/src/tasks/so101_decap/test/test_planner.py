from pathlib import Path

import numpy as np
import pytest

from so101_decap.planner import DecapConfig, DecapPlanner, Unreachable, rotate_about_vertical
from so101_manipulation.kinematics import ArmKinematics

URDF = Path(__file__).resolve().parents[3] / "platform" / "so101_description" / "urdf" / "so101.urdf"


@pytest.fixture(scope="module")
def kin():
    return ArmKinematics(URDF)


@pytest.fixture(scope="module")
def plan(kin):
    return DecapPlanner(kin).plan()


def test_rotation_about_the_cap_axis():
    T = np.eye(4)
    T[:3, 3] = [0.25, 0.0, 0.05]
    out = rotate_about_vertical(T, (0.20, 0.0), np.pi / 2, dz=0.001)
    assert np.allclose(out[:3, 3], [0.20, 0.05, 0.051])
    assert np.allclose(out[:3, 0], [0, 1, 0])


def test_strokes_cover_the_turns(plan):
    cfg = DecapConfig()
    assert len(plan.strokes) == int(np.ceil(cfg.unscrew_turns * 360 / cfg.stroke_deg))


def test_strokes_keep_the_grasp_on_the_cap_axis(kin, plan):
    """Every point of a stroke keeps the jaws at the same place around the cap,
    rising with the thread."""
    cfg = DecapConfig()
    axis = np.array(cfg.fixture_xy)
    r0 = np.linalg.norm(kin.tcp_position(plan.cap_grasp)[:2] - axis)
    z0 = kin.tcp_position(plan.cap_grasp)[2]
    for stroke in plan.strokes:
        for q in stroke.turn:
            p = kin.tcp_position(q)
            assert abs(np.linalg.norm(p[:2] - axis) - r0) < 0.002
        T = kin.fk(stroke.turn[-1])
        assert T[2, 2] < -0.999                         # still pointing down
        assert kin.tcp_position(stroke.turn[-1])[2] > z0  # rising with the thread


def test_wrist_stays_within_its_limits(kin, plan):
    for stroke in plan.strokes:
        for q in stroke.turn + [stroke.back]:
            assert kin.lower[4] - 1e-6 <= q[4] <= kin.upper[4] + 1e-6


def test_bottle_ends_above_the_box(kin, plan):
    cfg = DecapConfig()
    place = kin.tcp_position(plan.bottle_place)
    assert np.linalg.norm(place[:2] - cfg.bottle_box_xy) < 0.03   # TCP is offset from the bottle axis
    assert kin.fk(plan.bottle_above_box)[2, 2] < -np.cos(np.radians(10))  # carried upright (< 10 deg)


def test_fixture_out_of_reach(kin):
    with pytest.raises(Unreachable):
        DecapPlanner(kin, DecapConfig(fixture_xy=(0.45, 0.0))).plan()
