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


@pytest.mark.parametrize("xy", [(0.22, 0.13), (0.17, 0.0 + 0.10), (0.26, -0.12), (0.16, 0.20), (0.28, 0.05)])
def test_pick_and_insertion_plan(kin, xy):
    cfg = DecapConfig()
    pick = DecapPlanner(kin).plan_pick(xy)
    # the bottle's axis ends on the fixture axis at every search angle
    planner = DecapPlanner(kin)
    for ins in pick.inserts:
        axis = kin.tcp_position(ins.q) + planner.held_offset(ins.q, cfg.neck_diameter)
        assert np.linalg.norm(axis[:2] - cfg.fixture_xy) < 0.003
    assert len(pick.inserts) == len(cfg.search_deg)
    # the search covers the 72 deg period of the feet in steps smaller than the ~23 deg window
    angles = sorted(i.angle_deg for i in pick.inserts)
    assert angles[-1] - angles[0] >= 72 - 12 and max(np.diff(angles)) <= 12


def test_bottle_out_of_reach(kin):
    with pytest.raises(Unreachable, match="outside"):
        DecapPlanner(kin).plan_pick((0.40, 0.0))
