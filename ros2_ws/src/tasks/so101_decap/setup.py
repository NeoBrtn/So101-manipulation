from glob import glob

from setuptools import setup

package_name = "so101_decap"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    zip_safe=True,
    description="Phase 2 (design stage): unscrew the cap of a bottle held in a fixture.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "decap = so101_decap.decap_node:main",
        ],
    },
)
