#!/usr/bin/env python3
"""
qr_coordinates_node.py
----------------------
ROS 2 Humble node that detects QR codes from an Intel RealSense D435 and reports
each QR code's ID plus its position.

Two modes (set the "mode" parameter):
  * "2d"   : COLOR ONLY. Publishes the QR ID and its 2D pixel position (u, v).
             Works without depth — handy on a Pi 4B where the depth stream can't
             start (e.g. the camera browns out without a powered USB hub).
  * "3d"   : COLOR + ALIGNED DEPTH + camera_info. Publishes the QR ID and the
             real 3D point (X, Y, Z) in meters, in the camera frame.
  * "auto" : Start in 2D; if depth frames are detected, upgrade to 3D
             automatically (default).

Published topics:
  ~/qr/id      std_msgs/String             decoded QR data (always)
  ~/qr/pixel   geometry_msgs/PointStamped  pixel position (x=u, y=v, z=0), 2D
  ~/qr/point   geometry_msgs/PointStamped  3D position in meters (3D only)

3D deprojection (pinhole model from camera_info):
    Z = depth(u, v)
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
Camera optical frame (meters): +X right, +Y down, +Z forward.
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

        # ---- Parameters ----
        self.declare_parameter("mode", "auto")  # "2d" | "3d" | "auto"
        self.declare_parameter(
            "color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("sync_slop", 0.05)

        self.mode = str(self.get_parameter("mode").value).lower()
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        slop = float(self.get_parameter("sync_slop").value)

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # Latest camera intrinsics (filled from camera_info when available).
        self._intrinsics = None  # (fx, fy, cx, cy)
        # Whether we've seen depth frames (for "auto" mode upgrade).
        self._depth_seen = False
        self._frames_seen = 0

        # ---- Publishers ----
        self.id_pub = self.create_publisher(String, "~/qr/id", 10)
        self.pixel_pub = self.create_publisher(PointStamped, "~/qr/pixel", 10)
        self.point_pub = self.create_publisher(PointStamped, "~/qr/point", 10)

        # Always listen to camera_info (cheap) so 3D can work when depth exists.
        self.create_subscription(
            CameraInfo, info_topic, self._on_info, 10)

        want_depth = self.mode in ("3d", "auto")

        if want_depth:
            # Time-synchronized color + aligned depth.
            color_sub = message_filters.Subscriber(self, Image, color_topic)
            depth_sub = message_filters.Subscriber(self, Image, depth_topic)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=10, slop=slop)
            self.sync.registerCallback(self._on_color_depth)

        if self.mode in ("2d", "auto"):
            # Plain color subscription — fires even if depth never arrives.
            self.create_subscription(
                Image, color_topic, self._on_color_only, 10)

        # Heartbeat so the user gets feedback even before any QR is detected.
        self.create_timer(3.0, self._heartbeat)

        self.get_logger().info(
            "qr_coordinates_node started.\n"
            f"  mode:        {self.mode}\n"
            f"  color:       {color_topic}\n"
            f"  depth:       {depth_topic} (used in 3d/auto)\n"
            f"  camera_info: {info_topic}\n"
            "Point a QR code at the D435. Publishing ~/qr/id, ~/qr/pixel"
            " (and ~/qr/point in 3d)."
        )

    # ---------------- callbacks ----------------

    def _on_info(self, msg: CameraInfo):
        k = msg.k
        if k[0] != 0.0 and k[4] != 0.0:
            self._intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def _heartbeat(self):
        if self._frames_seen == 0:
            self.get_logger().warn(
                "No color frames received yet — is the camera streaming?")
        else:
            depth_note = "with depth" if self._depth_seen else "color only (2D)"
            self.get_logger().info(
                f"Alive: {self._frames_seen} color frames seen, {depth_note}.")

    def _detect(self, color_image):
        """Return list of (id, u, v, corner_pts)."""
        results = []
        ok, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(
            color_image)
        if not ok or points is None:
            return results
        h, w = color_image.shape[:2]
        for qr_id, corner_pts in zip(decoded_info, points):
            corner_pts = corner_pts.astype(int)
            u = int(np.clip(int(np.mean(corner_pts[:, 0])), 0, w - 1))
            v = int(np.clip(int(np.mean(corner_pts[:, 1])), 0, h - 1))
            results.append((qr_id if qr_id else "(unreadable)", u, v, corner_pts))
        return results

    def _publish_id_pixel(self, header, label, u, v):
        self.id_pub.publish(String(data=label))
        px = PointStamped()
        px.header = header
        px.point.x = float(u)
        px.point.y = float(v)
        px.point.z = 0.0
        self.pixel_pub.publish(px)

    def _on_color_only(self, color_msg):
        # In "auto" mode, once depth is flowing we let the synced callback do
        # the work (3D) and skip the color-only path to avoid double-publishing.
        if self.mode == "auto" and self._depth_seen:
            return
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge (color) failed: {exc}")
            return
        self._frames_seen += 1

        for label, u, v, _ in self._detect(color):
            self._publish_id_pixel(color_msg.header, label, u, v)
            self.get_logger().info(
                f"QR ID: {label!r} | pixel (u, v) = ({u}, {v})  [2D]")

    def _on_color_depth(self, color_msg, depth_msg):
        self._depth_seen = True
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge (color+depth) failed: {exc}")
            return
        self._frames_seen += 1

        for label, u, v, _ in self._detect(color):
            # Always publish ID + pixel.
            self._publish_id_pixel(color_msg.header, label, u, v)

            # Try 3D if we have depth + intrinsics.
            if self._intrinsics is None:
                self.get_logger().info(
                    f"QR ID: {label!r} | pixel ({u}, {v}) — awaiting camera_info")
                continue

            raw = float(depth[v, u])
            z = raw / 1000.0 if depth.dtype == np.uint16 else raw
            if z <= 0.0:
                self.get_logger().info(
                    f"QR ID: {label!r} | pixel ({u}, {v}) — no valid depth")
                continue

            fx, fy, cx, cy = self._intrinsics
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            pt = PointStamped()
            pt.header = color_msg.header
            pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
            self.point_pub.publish(pt)
            self.get_logger().info(
                f"QR ID: {label!r} | X: {x:+.3f}  Y: {y:+.3f}  Z: {z:+.3f} m  [3D]")


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
