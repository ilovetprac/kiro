#!/usr/bin/env python3
"""
Launch the D435 with the IR (infra1) stream + depth aligned to infra1, and the
ArUco IR+depth node that prints the marker's X, Y (Cartesian, mm) continuously.

Usage:
    ros2 launch qr_coordinates_ros2 aruco_ir_depth.launch.py

Options:
    aruco_dictionary:=DICT_4X4_250     # match your markers (4x4 inner grid)
    emitter_enabled:=0                 # IR projector OFF (cleaner marker) / 1 ON
    start_camera:=false                # if the camera is already running

NOTE: this uses the DEPTH stream, which on a Raspberry Pi 4B needs adequate
power (a powered USB 3.0 hub) or the camera may brown out — see
TROUBLESHOOTING_PI.md. IR+depth draws less than RGB+depth, so it may work where
RGB+depth did not; try it.
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
    aruco_dictionary = LaunchConfiguration("aruco_dictionary")
    emitter_enabled = LaunchConfiguration("emitter_enabled")

    declare_start_camera = DeclareLaunchArgument(
        "start_camera", default_value="true",
        description="Also start the realsense2_camera driver.")
    declare_aruco_dict = DeclareLaunchArgument(
        "aruco_dictionary", default_value="DICT_4X4_250",
        description="ArUco dictionary of your markers (4x4 inner grid).")
    declare_emitter = DeclareLaunchArgument(
        "emitter_enabled", default_value="0",
        description="IR projector: 0 = OFF (cleaner ArUco), 1 = ON.")

    # RealSense driver: infra1 (IR) + depth aligned to infra1. RGB off.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch", "rs_launch.py",
            ])
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            # Align depth to the infra1 (left IR) frame.
            "align_depth.enable": "true",
            "enable_depth": "true",
            "enable_infra1": "true",
            "enable_infra2": "false",
            "enable_color": "false",
            # IR projector off by default -> cleaner marker in the IR image.
            "depth_module.emitter_enabled": emitter_enabled,
            # Modest, confirmed-supported profiles for the Pi 4B.
            "depth_module.depth_profile": "640,480,30",
            "depth_module.infra_profile": "640,480,30",
            # Trim unused streams / startup contention.
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_sync": "false",
            "initial_reset": "false",
        }.items(),
    )

    node = Node(
        package="qr_coordinates_ros2",
        executable="aruco_ir_depth_node",
        name="aruco_ir_depth_node",
        output="screen",
        parameters=[{
            "aruco_dictionary": aruco_dictionary,
            "ir_topic": "/camera/camera/infra1/image_rect_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_infra1/image_raw",
            "camera_info_topic": "/camera/camera/infra1/camera_info",
            "sync_slop": 0.05,
        }],
    )

    return LaunchDescription([
        declare_start_camera,
        declare_aruco_dict,
        declare_emitter,
        realsense_launch,
        node,
    ])
