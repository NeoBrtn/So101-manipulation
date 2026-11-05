"""perception.launch.py - the perception side of a task (perception container).

    ros2 launch so101_bringup perception.launch.py task:=pick_place

Includes share/so101_<task>/launch/perception.launch.py if the task has one.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction

from so101_bringup.tasks import arg, include_task_launch, task_share


def launch_setup(context):
    share = task_share(arg(context, "task"))
    actions = include_task_launch(share, "perception.launch.py", {"use_sim": arg(context, "use_sim")})
    return actions or [LogInfo(msg=f"Task '{arg(context, 'task')}' has no perception launch file.")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("task", default_value="pick_place"),
        DeclareLaunchArgument("use_sim", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
