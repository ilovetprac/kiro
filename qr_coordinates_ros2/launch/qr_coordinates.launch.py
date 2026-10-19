#!/usr/bin/env python3
"""
Launch the Intel RealSense D435 driver (with depth aligned to color) together
with the QR coordinates node.

Usage:
    ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py

Run only the QR node (camera already running elsewhere):
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
    mode = LaunchConfiguration("mode")
    enable_depth = LaunchConfiguration("enable_depth")

    declare_start_camera = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Also start the realsense2_camera driver.",
    )

    # Default to 2D (color only). 2D works without a powered USB hub; 3D needs
    # the depth stream (which on a Pi 4B requires adequate power for the IR
    # projector — see TROUBLESHOOTING_PI.md).
    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="2d",
        description="QR position mode: '2d' (color only), '3d', or 'auto'.",
    )

    # In 2D we don't start the depth stream at all (avoids the Pi 4B depth
    # brown-out). Set enable_depth:=true (or use mode:=3d/auto) once a powered
    # USB hub is in place.
    declare_enable_depth = DeclareLaunchArgument(
        "enable_depth",
        default_value="false",
        description="Start the depth stream. Leave false for 2D; true for 3D.",
    )

    # ArUco dictionary your markers belong to. Common: DICT_4X4_50,
    # DICT_5X5_100, DICT_6X6_250, DICT_APRILTAG_36h11.
    declare_aruco_dict = DeclareLaunchArgument(
        "aruco_dictionary",
        default_value="DICT_4X4_50",
        description="ArUco dictionary of your markers (e.g. DICT_4X4_50).",
    )

    # Start the RealSense driver with depth aligned to color (so the depth
    # pixel under the QR center matches the color pixel).
    #
    # NOTE: only keys that realsense2_camera's rs_launch.py actually declares
    # are forwarded here. 'start_camera' is OUR launch arg and is deliberately
    # NOT passed through (passing it caused the
    # "Parameter 'start_camera' is not supported" warning).
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
            "align_depth.enable": enable_depth,
            "enable_color": "true",
            "enable_depth": enable_depth,
            # Modest, confirmed-supported profiles — friendly for a Pi 4B.
            # (D435 supports 640x480x30 for both color and depth.)
            "rgb_camera.color_profile": "640,480,30",
            "depth_module.depth_profile": "640,480,30",
            # Reduce startup contention / IMU streams we don't need.
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            # IMPORTANT (Raspberry Pi 4B, RSUSB backend, FW 5.17.x):
            # enable_sync=true and initial_reset=true caused the device to fail
            # to start (null-pointer/block error, or a stuck state). With them
            # OFF the camera streams normally at ~26 Hz. The
            # "control_transfer ... Resource temporarily unavailable" lines are
            # harmless WARNINGS on the RSUSB backend and do NOT stop streaming.
            "enable_sync": "false",
            "initial_reset": "false",
        }.items(),
    )

    qr_node = Node(
        package="qr_coordinates_ros2",
        executable="qr_coordinates_node",
        name="qr_coordinates_node",
        output="screen",
        parameters=[{
            "mode": mode,
            "aruco_dictionary": LaunchConfiguration("aruco_dictionary"),
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "sync_slop": 0.05,
        }],
    )

    return LaunchDescription([
        declare_start_camera,
        declare_mode,
        declare_enable_depth,
        declare_aruco_dict,
        realsense_launch,
        qr_node,
    ])
