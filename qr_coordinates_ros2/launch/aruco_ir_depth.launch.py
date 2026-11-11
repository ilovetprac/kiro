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
    declare_target_id = DeclareLaunchArgument(
        "target_id", default_value="1",
        description="Only act on this marker ID; others ignored (-1 = any).")
    declare_emitter = DeclareLaunchArgument(
        "emitter_enabled", default_value="1",
        description="IR projector: 0 = OFF (cleaner ArUco), 1 = ON.")
    declare_preset = DeclareLaunchArgument(
        "depth_preset", default_value="High Accuracy",
        description="D400 depth visual preset. 'High Accuracy' gives fewer but "
                    "more trustworthy depth pixels — best for close, static, "
                    "precise work. Others: Default, High Density, Medium "
                    "Density, Hand.")

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
            # Depth visual preset. "High Accuracy" = conservative, high-
            # confidence depth (fewer but more trustworthy pixels) — best for
            # the close-range (<25 cm), static alignment this node performs.
            "depth_module.preset": LaunchConfiguration("depth_preset"),
            # Modest, confirmed-supported profiles for the Pi 4B.
            "depth_module.depth_profile": "640,480,30",
            "depth_module.infra_profile": "640,480,30",
            # Trim unused streams / startup contention.
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_sync": "false",
            # Keep initial_reset FALSE: on this Pi 4B + RSUSB + FW 5.17.x,
            # initial_reset:=true made the device fail to start. Instead, the
            # device is freed + USB-re-enumerated in run.sh BEFORE launch, which
            # clears the stuck control-transfer (errno 11) state safely.
            "initial_reset": "false",
            # Quiet the harmless "control_transfer ... Resource temporarily
            # unavailable" RSUSB WARNING flood so the marker output is readable.
            "log_level": "error",
        }.items(),
    )

    node = Node(
        package="qr_coordinates_ros2",
        executable="aruco_ir_depth_node",
        name="aruco_ir_depth_node",
        output="screen",
        # Force line-buffered, visible stdout so status prints (e.g. "move the
        # robot closer") appear immediately in the launch terminal.
        emulate_tty=True,
        parameters=[{
            "aruco_dictionary": aruco_dictionary,
            "target_id": LaunchConfiguration("target_id"),
            "ir_topic": "/camera/camera/infra1/image_rect_raw",
            "depth_topic": "/camera/camera/depth/image_rect_raw",
            "camera_info_topic": "/camera/camera/infra1/camera_info",
            "sync_slop": 0.05,
        }],
    )

    return LaunchDescription([
        declare_start_camera,
        declare_aruco_dict,
        declare_target_id,
        declare_emitter,
        declare_preset,
        realsense_launch,
        node,
    ])
