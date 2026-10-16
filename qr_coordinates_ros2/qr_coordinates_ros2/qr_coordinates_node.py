#!/usr/bin/env python3
"""
qr_coordinates_node.py
----------------------
ROS 2 Humble node that detects QR codes in an Intel RealSense D435 color stream
and publishes each QR code's X, Y, Z position (meters) relative to the camera,
together with the decoded QR ID.

Pipeline
========
  1. Subscribe (time-synchronized) to three topics from realsense2_camera:
       - color image            (sensor_msgs/Image)
       - aligned depth-to-color (sensor_msgs/Image, 16UC1, millimeters)
       - color camera_info      (sensor_msgs/CameraInfo, intrinsics)
  2. Detect & decode QR codes in the color image (OpenCV QRCodeDetector).
  3. For each QR code, take the center pixel, read the aligned-depth value
     there, and deproject (pixel + depth) into a 3D point in meters using the
     pinhole model from camera_info:
         Z = depth
         X = (u - cx) * Z / fx
         Y = (v - cy) * Z / fy
  4. Publish results:
       - ~/qr/point   geometry_msgs/PointStamped  (3D position, latest code)
       - ~/qr/id      std_msgs/String             (decoded ID, latest code)
       - also logged to the ROS console each detection.

The deprojection uses only camera_info, so this node does NOT depend on
pyrealsense2 (handy on ARM/Raspberry Pi where pyrealsense2 has no pip wheel).

Camera coordinate frame (REP 103 / optical frame, meters):
    +X : right, +Y : down, +Z : forward (into the scene)

Default topics assume realsense2_camera's Humble namespace /camera/camera.
Override with ROS parameters if your launch uses a different namespace.
"""

import numpy as np
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class QrCoordinatesNode(Node):
    def __init__(self):
        super().__init__("qr_coordinates_node")

        # ---- Parameters (override at launch / CLI) ----
        self.declare_parameter(
            "color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info")
        # Max sync time difference between color/depth/info (seconds).
        self.declare_parameter("sync_slop", 0.05)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        slop = float(self.get_parameter("sync_slop").value)

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # ---- Publishers ----
        self.point_pub = self.create_publisher(PointStamped, "~/qr/point", 10)
        self.id_pub = self.create_publisher(String, "~/qr/id", 10)

        # ---- Time-synchronized subscribers ----
        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub], queue_size=10, slop=slop
        )
        self.sync.registerCallback(self.on_frames)

        self.get_logger().info(
            "qr_coordinates_node started.\n"
            f"  color:       {color_topic}\n"
            f"  depth:       {depth_topic}\n"
            f"  camera_info: {info_topic}\n"
            "Point a QR code at the D435; publishing ~/qr/point and ~/qr/id."
        )

    def on_frames(self, color_msg, depth_msg, info_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            # Aligned depth is 16UC1 in millimeters.
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        # Intrinsics from CameraInfo K = [fx 0 cx; 0 fy cy; 0 0 1]
        k = info_msg.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn("camera_info has zero focal length; skipping.")
            return

        h, w = depth.shape[:2]

        ok, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(color)
        if not ok or points is None:
            return

        for qr_id, corner_pts in zip(decoded_info, points):
            corner_pts = corner_pts.astype(int)
            u = int(np.mean(corner_pts[:, 0]))
            v = int(np.mean(corner_pts[:, 1]))
            u = max(0, min(u, w - 1))
            v = max(0, min(v, h - 1))

            # Depth in meters. Aligned depth image is uint16 millimeters.
            raw = float(depth[v, u])
            z = raw / 1000.0 if depth.dtype == np.uint16 else raw

            label = qr_id if qr_id else "(unreadable)"

            if z <= 0.0:
                self.get_logger().info(
                    f"QR ID: {label!r} | no valid depth at pixel ({u}, {v})")
                continue

            # Pinhole deprojection: pixel + depth -> 3D point (camera frame).
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            self.get_logger().info(
                f"QR ID: {label!r} | X: {x:+.3f} m  Y: {y:+.3f} m  Z: {z:+.3f} m")

            # Publish the decoded ID.
            id_msg = String()
            id_msg.data = label
            self.id_pub.publish(id_msg)

            # Publish the 3D point, stamped in the color optical frame.
            pt = PointStamped()
            pt.header = color_msg.header  # same frame_id & stamp as the color image
            pt.point.x = float(x)
            pt.point.y = float(y)
            pt.point.z = float(z)
            self.point_pub.publish(pt)


def main(args=None):
    rclpy.init(args=args)
    node = QrCoordinatesNode()
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
