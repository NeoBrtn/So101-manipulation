from glob import glob

from setuptools import setup

package_name = "so101_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    zip_safe=True,
    description="Launch files of the SO-101 platform and the task plug-in convention.",
    license="Apache-2.0",
)
