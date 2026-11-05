"""robot_state_publisher for the SO-ARM101.

    ros2 launch so101_description viz.launch.py            # waits for /joint_states
    ros2 launch so101_description viz.launch.py demo:=true # joints frozen at 0 (smoke test)

Publishes /robot_description and the TFs of the arm from /joint_states (published
by so101_driver). Without the driver, the joint TFs do not exist and the arm appears
"in pieces" in RViz: demo:=true publishes a fake zero pose to check the display chain.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
    robot_description = urdf.read_text()

    return LaunchDescription([
        DeclareLaunchArgument("demo", default_value="false",
                              description="Publish fake joint_states (zero pose)"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # world → base_link: same as the MuJoCo frame (robot base at the world origin)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        ),
        # Smoke test only — in normal use, /joint_states comes from the driver_node.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("demo")),
        ),
    ])
