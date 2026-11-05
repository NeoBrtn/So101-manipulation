
from setuptools import setup

package_name = "so101_driver"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    description="ROS 2 driver of the SO-101 (MuJoCo sim or real arm), real camera and calibration tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "driver = so101_driver.driver_node:main",
            "camera_stream = so101_driver.camera_stream_node:main",
            "calibrate_camera = so101_driver.calibrate_camera:main",
            "measure_zero = so101_driver.measure_zero:main",
        ],
    },
)
