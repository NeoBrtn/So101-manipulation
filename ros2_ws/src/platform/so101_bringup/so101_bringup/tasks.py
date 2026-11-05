"""Task plug-in convention, used by the bringup launch files.

A task is a ROS package named ``so101_<task>``. It may install, in its share directory:

    config/driver.yaml            parameters of so101_driver for this task (simulation
                                  scene, published camera, object placement), as a ROS
                                  parameters file for the node ``so101_driver``
    launch/control.launch.py      its control-side nodes (control container)
    launch/perception.launch.py   its perception nodes (perception container)

Both launch files receive ``use_sim`` ("true" / "false"). Nothing else in the platform
knows about tasks: adding one is adding a package.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def task_share(task: str) -> Path:
    try:
        return Path(get_package_share_directory(f"so101_{task}"))
    except PackageNotFoundError:
        raise RuntimeError(f"Unknown task '{task}': no package so101_{task} in the workspace") from None


def task_driver_params(share: Path) -> dict:
    path = share / "config" / "driver.yaml"
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    return dict(data.get("so101_driver", {}).get("ros__parameters", {}))


def include_task_launch(share: Path, name: str, launch_arguments: dict) -> list:
    path = share / "launch" / name
    if not path.is_file():
        return []
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                     launch_arguments=launch_arguments.items())]
