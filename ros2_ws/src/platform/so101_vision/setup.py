from setuptools import setup

package_name = "so101_vision"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    description="Camera geometry, colour segmentation and tracking, shared by every task.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
