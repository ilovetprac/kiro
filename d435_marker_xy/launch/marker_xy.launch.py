import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("d435_marker_xy"),
        "config",
        "params.yaml",
    )

    return LaunchDescription([
        Node(
            package="d435_marker_xy",
            executable="marker_xy_node",
            name="d435_marker_xy",
            output="screen",
            emulate_tty=True,
            parameters=[params],
        ),
    ])
