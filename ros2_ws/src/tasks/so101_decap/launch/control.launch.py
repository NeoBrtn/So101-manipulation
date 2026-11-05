"""Phase 2, control side (included by so101_bringup). Perception (YOLO) comes later."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

CONFIG = Path(get_package_share_directory("so101_decap")) / "config" / "decap.yaml"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        Node(package="so101_decap", executable="decap", name="decap", output="screen",
             parameters=[str(CONFIG)]),
    ])
