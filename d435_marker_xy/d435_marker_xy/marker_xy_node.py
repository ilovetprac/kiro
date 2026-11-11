#!/usr/bin/env python3
"""
ROS 2 Humble node: continuously publish an ArUco marker's X, Y position
(in millimeters) relative to an Intel RealSense D435, using the DEPTH and
INFRARED streams only (no RGB).

Marker center = ORIGIN of a cartesian plane
-------------------------------------------
When the camera is pointed straight at the marker (marker on the optical
axis / image center), the readout is (X=0, Y=0).
    X: right +, left -
    Y: up +, down -
    Units: millimeters (mm)

How it works
------------
  - Detection uses the LEFT INFRARED (IR) stream. The D435 IR imager sees the
    printed black/white ArUco pattern; the RGB stream is never enabled.
  - Distance uses the DEPTH stream, aligned to the IR frame.
  - The marker-center pixel + its depth are deprojected to a 3D point using the
    camera intrinsics. Because that point is measured from the optical axis,
    "camera centered on marker" => (0, 0) with no manual calibration.

D435 note: the IR projector ("emitter") sprays a dot pattern that corrupts the
ArUco pattern in the IR image, so the emitter is turned OFF by default (param
'emitter_enabled'). Depth still works from ambient light indoors. Enable it if
you need the projector for depth range (detection may degrade).

Your marker
-----------
A DICT_4X4_50 marker whose full printed square (incl. white border) is 10 cm.
The 4x4 refers to the inner code grid; the border is normal for every ArUco
marker. 'marker_size_mm' (default 100.0) is only used for the optional pose/TF
output; the X/Y readout comes straight from depth and does not need it.

Published topics
----------------
  ~/marker_point   (geometry_msgs/PointStamped)  X,Y,Z in METERS (ROS standard
                   SI units; +X right, +Y up, +Z forward), frame = camera.
  ~/marker_xy_mm   (std_msgs/Float32MultiArray)   [X_mm, Y_mm, Z_mm].
  ~/marker_found   (std_msgs/Bool)                True when the target marker is
                   currently detected with valid depth.

Parameters (ros2 param / launch)
--------------------------------
  aruco_dict        string  default "DICT_4X4_50"
  marker_id         int     default -1  (>=0 tracks only that ID; -1 = first seen)
  width/height/fps  int     stream config (default 640/480/30)
  emitter_enabled   bool    default False (IR projector off for clean detection)
  depth_window      int     default 2  (half-window for median depth filtering)
  publish_rate_hz   double  default 30.0
  frame_id          string  default "d435_ir_optical_frame"
  marker_size_mm    double  default 100.0 (full square incl. border)
  log_to_console    bool    default True (print a live X,Y line to the log)

Run
---
  ros2 run d435_marker_xy marker_xy_node
  ros2 run d435_marker_xy marker_xy_node --ros-args -p marker_id:=0
  ros2 launch d435_marker_xy marker_xy.launch.py
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32MultiArray

try:
    import pyrealsense2 as rs
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        "pyrealsense2 is not installed. On the Pi: pip3 install pyrealsense2"
    ) from e

try:
    import cv2
    import cv2.aruco as aruco
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        "OpenCV with aruco not installed. On the Pi: pip3 install opencv-contrib-python"
    ) from e


def make_aruco_detector(dict_name: str):
    """Return detect(gray)->(corners, ids), compatible with OpenCV >=4.7 and <=4.6."""
    if not hasattr(aruco, dict_name):
        valid = [a for a in dir(aruco) if a.startswith("DICT_")]
        raise SystemExit(
            f"Unknown ArUco dictionary '{dict_name}'. Options include: {', '.join(valid)}"
        )
    dict_id = getattr(aruco, dict_name)

    if hasattr(aruco, "ArucoDetector"):  # OpenCV >= 4.7
        dictionary = aruco.getPredefinedDictionary(dict_id)
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, params)

        def detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
        return detect

    # OpenCV <= 4.6 (e.g. ROS 2 Humble's system OpenCV 4.5.x)
    dictionary = aruco.Dictionary_get(dict_id)
    params = aruco.DetectorParameters_create()

    def detect(gray):
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
        return corners, ids
    return detect


class MarkerXYNode(Node):
    def __init__(self):
        super().__init__("d435_marker_xy")

        # ---- parameters ----
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("marker_id", -1)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("emitter_enabled", False)
        self.declare_parameter("depth_window", 2)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("frame_id", "d435_ir_optical_frame")
        self.declare_parameter("marker_size_mm", 100.0)
        self.declare_parameter("log_to_console", True)

        gp = self.get_parameter
        self.aruco_dict = gp("aruco_dict").value
        self.marker_id = int(gp("marker_id").value)
        self.width = int(gp("width").value)
        self.height = int(gp("height").value)
        self.fps = int(gp("fps").value)
        self.emitter_enabled = bool(gp("emitter_enabled").value)
        self.depth_window = int(gp("depth_window").value)
        self.publish_rate_hz = float(gp("publish_rate_hz").value)
        self.frame_id = gp("frame_id").value
        self.marker_size_mm = float(gp("marker_size_mm").value)
        self.log_to_console = bool(gp("log_to_console").value)

        self.detect = make_aruco_detector(self.aruco_dict)

        # ---- publishers ----
        self.pub_point = self.create_publisher(PointStamped, "~/marker_point", 10)
        self.pub_xy = self.create_publisher(Float32MultiArray, "~/marker_xy_mm", 10)
        self.pub_found = self.create_publisher(Bool, "~/marker_found", 10)

        # ---- RealSense setup ----
        self._start_realsense()

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0 else 0.0
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            "d435_marker_xy started. Depth + IR only (no RGB). "
            f"dict={self.aruco_dict}, marker_id="
            f"{'first seen' if self.marker_id < 0 else self.marker_id}. "
            "Marker center = origin (0,0). X: right+, Y: up+. Units: mm."
        )

    def _start_realsense(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        # infrared index 1 = left IR imager (y8) on the D435
        config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)

        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(
                rs.option.emitter_enabled, 1.0 if self.emitter_enabled else 0.0)
            self.get_logger().info(
                f"IR emitter: {'ON' if self.emitter_enabled else 'OFF'}")
        self.depth_scale = depth_sensor.get_depth_scale()  # meters/unit

        self.align = rs.align(rs.stream.infrared)

        ir_profile = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        self.intr = ir_profile.get_intrinsics()
        self.get_logger().info(
            f"Intrinsics fx={self.intr.fx:.1f} fy={self.intr.fy:.1f} "
            f"cx={self.intr.ppx:.1f} cy={self.intr.ppy:.1f} "
            f"({self.intr.width}x{self.intr.height}), depth_scale={self.depth_scale}")

    def _tick(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError:
            self.get_logger().warn("No frames from RealSense (timeout).", throttle_duration_sec=2.0)
            return

        frames = self.align.process(frames)
        ir_frame = frames.get_infrared_frame(1)
        depth_frame = frames.get_depth_frame()
        if not ir_frame or not depth_frame:
            return

        ir_image = np.asanyarray(ir_frame.get_data())
        corners, ids = self.detect(ir_image)

        target = None  # (cx_px, cy_px, id)
        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            chosen = None
            if self.marker_id >= 0:
                for i, mid in enumerate(ids_flat):
                    if int(mid) == self.marker_id:
                        chosen = i
                        break
            else:
                chosen = 0
            if chosen is not None:
                c = corners[chosen].reshape(4, 2)
                target = (float(c[:, 0].mean()), float(c[:, 1].mean()), int(ids_flat[chosen]))

        now = self.get_clock().now().to_msg()

        if target is None:
            self.pub_found.publish(Bool(data=False))
            if self.log_to_console:
                self.get_logger().info("No marker detected.", throttle_duration_sec=1.0)
            return

        cx_px, cy_px, mid = target

        # median depth over a small window to ride through holes/noise
        win = self.depth_window
        px, py = int(round(cx_px)), int(round(cy_px))
        x0, x1 = max(0, px - win), min(self.intr.width, px + win + 1)
        y0, y1 = max(0, py - win), min(self.intr.height, py + win + 1)
        depth_img = np.asanyarray(depth_frame.get_data())
        patch = depth_img[y0:y1, x0:x1].astype(np.float32)
        valid = patch[patch > 0]
        if valid.size == 0:
            self.pub_found.publish(Bool(data=False))
            if self.log_to_console:
                self.get_logger().info(
                    f"Marker {mid} seen but no valid depth at center.",
                    throttle_duration_sec=1.0)
            return

        depth_m = float(np.median(valid)) * self.depth_scale

        # deproject to 3D (meters) in the optical frame: +X right, +Y down, +Z fwd
        X_m, Y_m, Z_m = rs.rs2_deproject_pixel_to_point(self.intr, [cx_px, cy_px], depth_m)

        # cartesian-plane convention for output: +Y up
        X_up, Y_up, Z_fwd = X_m, -Y_m, Z_m
        X_mm, Y_mm, Z_mm = X_up * 1000.0, Y_up * 1000.0, Z_fwd * 1000.0

        # PointStamped in meters (ROS standard SI), +Y up
        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = self.frame_id
        pt.point.x = X_up
        pt.point.y = Y_up
        pt.point.z = Z_fwd
        self.pub_point.publish(pt)

        arr = Float32MultiArray()
        arr.data = [float(X_mm), float(Y_mm), float(Z_mm)]
        self.pub_xy.publish(arr)

        self.pub_found.publish(Bool(data=True))

        if self.log_to_console:
            self.get_logger().info(
                f"Marker {mid}:  X={X_mm:+8.1f} mm  Y={Y_mm:+8.1f} mm  (Z={Z_mm:7.1f} mm)",
                throttle_duration_sec=0.0)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerXYNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
