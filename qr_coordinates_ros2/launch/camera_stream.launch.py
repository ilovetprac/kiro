#!/usr/bin/env python3
"""
camera_stream.launch.py
-----------------------
Publish the live D435 camera feed from the Raspberry Pi over WiFi so it can be
SUBSCRIBED TO and PROCESSED on the mini PC. Both machines run ROS 2 Humble, so
discovery is native (same ROS_DOMAIN_ID — no DDS tricks needed).

Why this exists
===============
Raw image topics are far too heavy for WiFi (640x480 mono @ 30 Hz ~ 9 MB/s;
RGB ~ 27 MB/s) and will drop/stall. This launch starts the camera and a
`image_transport republish` node that re-publishes the stream as JPEG
(sensor_msgs/CompressedImage), which is small enough to stream reliably.

The mini PC subscribes to the COMPRESSED topic and decompresses locally back
to a normal sensor_msgs/Image for processing — see NETWORK_STREAM.md for the
exact commands and the (simple) ROS_DOMAIN_ID setup.

Topics published by this launch (on the Pi)
============================================
  <image_topic>                 sensor_msgs/Image           raw (local use)
  <image_topic>/compressed      sensor_msgs/CompressedImage JPEG (stream this!)
  (depth is also available raw: /camera/camera/depth/image_rect_raw)

Usage (on the Pi)
=================
    ros2 launch qr_coordinates_ros2 camera_stream.launch.py

Options:
    stream_source:=infra1   # which stream to compress+stream: infra1 | color | depth
    jpeg_quality:=30        # 1..100 ; lower = smaller/faster over WiFi
    start_camera:=false     # if the camera driver is already running
    emitter_enabled:=1      # IR projector on/off

NOTE on bandwidth: JPEG quality 20-40 is usually the sweet spot over WiFi.
Start low (e.g. 30) and raise only if you need more detail and the link keeps up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    emitter_enabled = LaunchConfiguration("emitter_enabled")
    stream_source = LaunchConfiguration("stream_source")
    jpeg_quality = LaunchConfiguration("jpeg_quality")

    declare_start_camera = DeclareLaunchArgument(
        "start_camera", default_value="true",
        description="Also start the realsense2_camera driver.")
    declare_emitter = DeclareLaunchArgument(
        "emitter_enabled", default_value="1",
        description="IR projector: 0 = OFF, 1 = ON.")
    declare_source = DeclareLaunchArgument(
        "stream_source", default_value="infra1",
        description="Which camera stream to compress + stream over WiFi: "
                    "infra1 (IR, default), color, or depth.")
    declare_quality = DeclareLaunchArgument(
        "jpeg_quality", default_value="30",
        description="JPEG quality 1..100. Lower = less bandwidth. 20-40 is a "
                    "good WiFi range.")

    # The raw image topic to compress. The D435 topics depend on which stream.
    # infra1 (IR) is what the alignment pipeline already uses; color is nicer to
    # look at but must be enabled (and draws more power). depth is 16-bit.
    #   infra1 -> /camera/camera/infra1/image_rect_raw
    #   color  -> /camera/camera/color/image_raw
    #   depth  -> /camera/camera/depth/image_rect_raw
    # We resolve the topic in a small Python expression via PythonExpression so
    # a single stream_source arg controls both the camera config and republish.
    image_topic = PythonExpression([
        "'/camera/camera/infra1/image_rect_raw' if '",
        stream_source,
        "' == 'infra1' else ('/camera/camera/color/image_raw' if '",
        stream_source,
        "' == 'color' else '/camera/camera/depth/image_rect_raw')",
    ])

    # Enable color only when the user asks to stream color.
    enable_color = PythonExpression(
        ["'true' if '", stream_source, "' == 'color' else 'false'"])
    enable_infra1 = PythonExpression(
        ["'false' if '", stream_source, "' == 'color' else 'true'"])

    # RealSense driver.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch", "rs_launch.py",
            ])
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "enable_depth": "true",
            "enable_infra1": enable_infra1,
            "enable_infra2": "false",
            "enable_color": enable_color,
            "depth_module.emitter_enabled": emitter_enabled,
            "depth_module.depth_profile": "640,480,30",
            "depth_module.infra_profile": "640,480,30",
            "rgb_camera.color_profile": "640,480,30",
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_sync": "false",
            "initial_reset": "false",
            "log_level": "error",
        }.items(),
    )

    # image_transport republish: raw -> compressed (JPEG). This is the node
    # that makes the feed small enough to stream over WiFi. The mini PC then
    # subscribes to "<image_topic>/compressed".
    #
    # ROS 2 Humble republish CLI form:
    #   ros2 run image_transport republish raw compressed \
    #       --ros-args -r in:=<image_topic> -r out:=<image_topic>
    # Here as a Node so JPEG quality is set via a parameter.
    # NOTE on JPEG quality: compressed_image_transport exposes it as a dynamic
    # parameter on the OUTPUT topic namespace, e.g.
    #   /<out>/compressed/jpeg_quality
    # which is awkward to set cleanly at launch across ROS 2 versions. We set a
    # sensible default here via `-p` on that namespaced name, and you can also
    # change it live with `ros2 param set` (see NETWORK_STREAM.md).
    republish = Node(
        package="image_transport",
        executable="republish",
        name="d435_stream_republish",
        output="screen",
        arguments=[
            "raw", "compressed",
            # jpeg_quality is read from the compressed publisher's params; pass
            # it namespaced so it applies to the output /compressed topic.
            "--ros-args",
            "-p", ["out.compressed.jpeg_quality:=", jpeg_quality],
        ],
        remappings=[
            ("in", image_topic),
            ("out", image_topic),
        ],
    )

    return LaunchDescription([
        declare_start_camera,
        declare_emitter,
        declare_source,
        declare_quality,
        realsense_launch,
        republish,
    ])
