from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_run_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    package_data={"robot_arcade": ["assets/*.png"]},
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Elena Villalba",
    maintainer_email="elena.villalba@upc.edu",
    description="ROS 2 bridge for the Robot Run teaching simulator.",
    license="Educational use within Intelligent Robotics at UPC-EEBE",
    entry_points={
        "console_scripts": [
            "robot_run = robot_run_ros.robot_run_node:main",
        ],
    },
)

