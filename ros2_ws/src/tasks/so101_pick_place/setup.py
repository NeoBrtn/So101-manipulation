from glob import glob

from setuptools import setup

package_name = "so101_pick_place"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    zip_safe=True,
    description="Phase 1: camera-guided pick & place of a cube into a box.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception = so101_pick_place.perception_node:main",
            "pick_place = so101_pick_place.pick_place_node:main",
            "replan_test = so101_pick_place.replan_test:main",
        ],
    },
)
