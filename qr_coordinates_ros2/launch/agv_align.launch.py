#!/usr/bin/env python3
"""
Full AGV alignment launch — NO Arduino. Starts:
  1. the D435 camera + ArUco IR/depth marker node (aruco_ir_depth.launch.py), and
  2. the Pi GPIO stepper node that drives the two microstep drivers directly.

Usage:
    ros2 launch qr_coordinates_ros2 agv_align.launch.py

Requires: sudo apt install python3-lgpio (no daemon needed).

Override pins/params, e.g.:
    ros2 launch qr_coordinates_ros2 agv_align.launch.py \
        invert_x:=true invert_y:=true steps_per_mm:=1600
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Marker-detection launch (camera + aruco_ir_depth_node), with dict/target
    # passed through from this launch's arguments.
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("qr_coordinates_ros2"),
                "launch", "aruco_ir_depth.launch.py",
            ])
        ),
        launch_arguments={
            "aruco_dictionary": LaunchConfiguration("aruco_dictionary"),
            "target_id": LaunchConfiguration("target_id"),
        }.items(),
    )

    # Pi GPIO stepper driver node.
    stepper = Node(
        package="qr_coordinates_ros2",
        executable="gpio_stepper_node",
        name="gpio_stepper_node",
        output="screen",
        parameters=[{
            "marker_topic": "/aruco_ir_depth_node/marker/xy_mm",
            "steps_per_mm": LaunchConfiguration("steps_per_mm"),
            "x_step_pin": 17, "x_dir_pin": 27,
            "y_step_pin": 22, "y_dir_pin": 23,
            "deadband_mm": 1.0,
            "fine_zone_mm": 5.0,
            "fast_sps": 4000,
            "fine_sps": 800,
            "max_travel_mm": 70.0,
            "invert_x": LaunchConfiguration("invert_x"),
            "invert_y": LaunchConfiguration("invert_y"),
            "lost_timeout_s": 0.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("steps_per_mm", default_value="1600.0"),
        DeclareLaunchArgument("invert_x", default_value="false"),
        DeclareLaunchArgument("invert_y", default_value="false"),
        DeclareLaunchArgument("aruco_dictionary", default_value="DICT_4X4_250"),
        DeclareLaunchArgument("target_id", default_value="1"),
        aruco_launch,
        stepper,
    ])
