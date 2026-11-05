"""sim.launch.py - everything in one process tree (no Docker split), for development.

    ros2 launch so101_bringup sim.launch.py task:=pick_place [viewer:=true] [rviz:=true]
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(get_package_share_directory("so101_bringup"))
    description = Path(get_package_share_directory("so101_description"))
    args = {"task": LaunchConfiguration("task"), "use_sim": "true"}

    def include(path, extra=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments={**args, **(extra or {})}.items())

    return LaunchDescription([
        DeclareLaunchArgument("task", default_value="pick_place"),
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        include(here / "launch" / "control.launch.py", {"viewer": LaunchConfiguration("viewer")}),
        include(here / "launch" / "perception.launch.py"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description / "launch" / "viz.launch.py"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(here / "rviz" / "so101.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
