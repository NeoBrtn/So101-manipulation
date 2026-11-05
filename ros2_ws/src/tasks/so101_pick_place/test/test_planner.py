from pathlib import Path

import numpy as np
import pytest

from so101_manipulation.kinematics import ArmKinematics
from so101_pick_place.planner import PlannerConfig, Unreachable, closing_direction, in_box, plan_pick_place

URDF = Path(__file__).resolve().parents[3] / "platform" / "so101_description" / "urdf" / "so101.urdf"
BOX = np.array([0.05, -0.25, 0.0])   # scene_tek5.xml


@pytest.fixture(scope="module")
def kin():
    return ArmKinematics(URDF)


@pytest.mark.parametrize("radius", [0.18, 0.24, 0.30])
@pytest.mark.parametrize("angle_deg", [-50, -10, 30, 60])
def test_plans_over_the_cube_spawn_area(kin, radius, angle_deg):
    a = np.radians(angle_deg)
    cube = [radius * np.cos(a), radius * np.sin(a), 0.025]
    plan = plan_pick_place(kin, cube, BOX)
    grasp = kin.tcp_position(plan.grasp)
    assert np.linalg.norm(grasp[:2] - cube[:2]) < 0.002
    assert abs(grasp[2] - PlannerConfig().grasp_height) < 0.002
    assert np.linalg.norm(kin.tcp_position(plan.drop)[:2] - BOX[:2]) < 0.01


def test_too_far_is_unreachable(kin):
    with pytest.raises(Unreachable, match="outside"):
        plan_pick_place(kin, [0.45, 0.0, 0.025], BOX)


def test_too_close_is_unreachable(kin):
    with pytest.raises(Unreachable):
        plan_pick_place(kin, [0.05, 0.02, 0.025], BOX)


def test_moving_jaw_opens_away_from_box():
    cube = np.array([0.15, -0.20])
    d = closing_direction(cube, BOX[:2])
    assert d @ (BOX[:2] - cube) >= 0
    assert abs(d @ cube) < 1e-9   # perpendicular to the radial direction


def test_in_box():
    assert in_box([0.06, -0.23, 0.02], BOX)
    assert not in_box([0.2, 0.0, 0.02], BOX)
