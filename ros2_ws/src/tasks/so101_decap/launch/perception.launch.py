"""Phase 2, perception side: the bottle, found by its cap (included by so101_bringup)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

CONFIG = Path(get_package_share_directory("so101_decap")) / "config" / "decap.yaml"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        Node(package="so101_decap", executable="perception", name="bottle_perception",
             output="screen", parameters=[str(CONFIG)]),
    ])
