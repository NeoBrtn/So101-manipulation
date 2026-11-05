from setuptools import setup

package_name = "so101_manipulation"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    description="Pinocchio kinematics, trajectory streaming and arm utilities of the SO-101.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_server = so101_manipulation.motion_server_node:main",
        ],
    },
)
