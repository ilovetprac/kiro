#!/usr/bin/env python3
"""
Launch the Intel RealSense D435 driver (with depth aligned to color) together
with the QR coordinates node.

Usage:
    ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py

Set start_camera:=false if you already run realsense2_camera elsewhere:
    ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py start_camera:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")

    declare_start_camera = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Also start the realsense2_camera driver.",
    )

    # Start the RealSense driver with depth aligned to color (needed so the
    # depth pixel under the QR center matches the color pixel).
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            # Modest resolution/fps — friendlier for a Raspberry Pi 4B.
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
        }.items(),
    )

    qr_node = Node(
        package="qr_coordinates_ros2",
        executable="qr_coordinates_node",
        name="qr_coordinates_node",
        output="screen",
        parameters=[{
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "sync_slop": 0.05,
        }],
    )

    return LaunchDescription([
        declare_start_camera,
        realsense_launch,
        qr_node,
    ])
