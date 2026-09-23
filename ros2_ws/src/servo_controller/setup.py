from glob import glob
from setuptools import find_packages, setup

package_name = "servo_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="uuav",
    maintainer_email="2365421405@qq.com",
    description="ROS 2 servo driver and waypoint trigger nodes for PCA9685/ServoKit.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_driver_node = servo_controller.servo_driver_node:main",
            "servo_waypoint_task = servo_controller.servo_waypoint_task:main",
        ],
    },
)
