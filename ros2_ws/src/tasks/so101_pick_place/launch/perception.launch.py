"""Phase 1, perception side: cube + drop box detection (included by so101_bringup)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

CONFIG = Path(get_package_share_directory("so101_pick_place")) / "config" / "pick_place.yaml"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        Node(package="so101_pick_place", executable="perception", name="perception",
             output="screen", parameters=[str(CONFIG)]),
    ])
