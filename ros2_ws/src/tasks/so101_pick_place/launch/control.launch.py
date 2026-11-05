"""Phase 1, control side: the pick & place state machine (included by so101_bringup)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG = Path(get_package_share_directory("so101_pick_place")) / "config" / "pick_place.yaml"


def launch_setup(context):
    real = LaunchConfiguration("use_sim").perform(context).lower() != "true"
    # Real arm: never moves on its own at launch, only on /pick_and_place
    overrides = {"home_on_start": False, "continuous": False} if real else {}
    return [Node(package="so101_pick_place", executable="pick_place", name="pick_place",
                 output="screen", parameters=[str(CONFIG), overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
