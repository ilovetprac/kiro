from setuptools import setup
import os
from glob import glob

package_name = "qr_coordinates_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install launch files
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ilovetprac",
    maintainer_email="you@example.com",
    description=(
        "ROS 2 Humble node that detects QR codes from an Intel RealSense D435 "
        "and publishes each code's X, Y, Z position (meters) and QR ID."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "qr_coordinates_node = qr_coordinates_ros2.qr_coordinates_node:main",
            "aruco_ir_depth_node = qr_coordinates_ros2.aruco_ir_depth_node:main",
            "serial_bridge_node = qr_coordinates_ros2.serial_bridge_node:main",
        ],
    },
)
