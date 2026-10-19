#!/usr/bin/env python3
"""
qr_coordinates_node.py
----------------------
ROS 2 Humble node that detects **ArUco markers** from an Intel RealSense D435
and reports each marker's integer ID plus its position.

(Despite the historical file/node name "qr_coordinates", this detects ArUco
markers, not QR codes. ArUco markers carry a numeric ID and are far more robust
to detect than QR codes.)

Modes (set the "mode" parameter):
  * "2d"   : COLOR ONLY. Publishes the marker ID and its 2D pixel center (u, v).
             Works without depth — handy on a Pi 4B where the depth stream can't
             start (e.g. the camera browns out without a powered USB hub).
  * "3d"   : COLOR + ALIGNED DEPTH + camera_info. Publishes the marker ID and
             the real 3D point (X, Y, Z) in meters, in the camera frame.
  * "auto" : Start in 2D; upgrade to 3D automatically if depth frames appear.

Parameters:
  mode                "2d" | "3d" | "auto"            (default "auto")
  aruco_dictionary    e.g. "DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250",
                      "DICT_APRILTAG_36h11"           (default "DICT_4X4_50")
  color_topic / depth_topic / camera_info_topic / sync_slop
  save_debug_image    bool, dump an annotated frame for inspection (default false)
  debug_image_path    where to save it (default /tmp/aruco_debug_frame.png)

Published topics:
  ~/marker/id     std_msgs/Int32              detected marker ID (always)
  ~/marker/pixel  geometry_msgs/PointStamped  pixel center (x=u, y=v, z=0), 2D
  ~/marker/point  geometry_msgs/PointStamped  3D position in meters (3D only)

3D deprojection (pinhole model from camera_info):
    Z = depth(u, v);  X = (u - cx) * Z / fx;  Y = (v - cy) * Z / fy
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
from std_msgs.msg import Int32


def _get_aruco_dictionary(name):
    """Return a cv2.aruco dictionary by name, handling both the new (4.7+) and
    old OpenCV ArUco APIs."""
    name = name.upper()
    const = getattr(cv2.aruco, name, None)
    if const is None:
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    # OpenCV >= 4.7
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(const)
    # Older OpenCV
    return cv2.aruco.Dictionary_get(const)


class ArucoCoordinatesNode(Node):
    def __init__(self):
        super().__init__("qr_coordinates_node")

        # ---- Parameters ----
        self.declare_parameter("mode", "auto")  # "2d" | "3d" | "auto"
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter(
            "color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("save_debug_image", False)
        self.declare_parameter(
            "debug_image_path", "/tmp/aruco_debug_frame.png")

        self.mode = str(self.get_parameter("mode").value).lower()
        dict_name = str(self.get_parameter("aruco_dictionary").value)
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        slop = float(self.get_parameter("sync_slop").value)
        self._save_debug = bool(self.get_parameter("save_debug_image").value)
        self._debug_path = str(self.get_parameter("debug_image_path").value)

        self.bridge = CvBridge()

        # ---- ArUco detector (handles both new and old OpenCV APIs) ----
        self._aruco_dict = _get_aruco_dictionary(dict_name)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, params)
            self._new_api = True
        else:
            self._params = cv2.aruco.DetectorParameters_create()
            self._detector = None
            self._new_api = False

        # State
        self._intrinsics = None  # (fx, fy, cx, cy)
        self._depth_seen = False
        self._frames_seen = 0
        self._detections = 0
        self._last_annotated = None

        # ---- Publishers ----
        self.id_pub = self.create_publisher(Int32, "~/marker/id", 10)
        self.pixel_pub = self.create_publisher(
            PointStamped, "~/marker/pixel", 10)
        self.point_pub = self.create_publisher(
            PointStamped, "~/marker/point", 10)

        # camera_info is cheap — always listen so 3D works when depth exists.
        self.create_subscription(CameraInfo, info_topic, self._on_info, 10)

        if self.mode in ("3d", "auto"):
            color_sub = message_filters.Subscriber(self, Image, color_topic)
            depth_sub = message_filters.Subscriber(self, Image, depth_topic)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=10, slop=slop)
            self.sync.registerCallback(self._on_color_depth)

        if self.mode in ("2d", "auto"):
            self.create_subscription(
                Image, color_topic, self._on_color_only, 10)

        self.create_timer(3.0, self._heartbeat)

        self.get_logger().info(
            "aruco marker node started.\n"
            f"  mode:        {self.mode}\n"
            f"  dictionary:  {dict_name}\n"
            f"  color:       {color_topic}\n"
            f"  depth:       {depth_topic} (used in 3d/auto)\n"
            f"  camera_info: {info_topic}\n"
            "Point an ArUco marker at the D435. Publishing ~/marker/id,"
            " ~/marker/pixel (and ~/marker/point in 3d)."
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
            return
        depth_note = "with depth" if self._depth_seen else "color only (2D)"
        self.get_logger().info(
            f"Alive: {self._frames_seen} color frames, {depth_note}, "
            f"{self._detections} marker detections so far.")
        if self._detections == 0:
            self.get_logger().warn(
                "Frames arriving but NO ArUco marker detected. Check the "
                "'aruco_dictionary' param matches your markers (default "
                "DICT_4X4_50). Try a larger marker, 30-80 cm away, flat and "
                "well-lit. Set save_debug_image:=true to dump a frame.")
            if self._save_debug and self._last_annotated is not None:
                try:
                    cv2.imwrite(self._debug_path, self._last_annotated)
                    self.get_logger().info(
                        f"Saved debug frame to {self._debug_path}")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"Could not save debug frame: {exc}")

    def _detect(self, color_image):
        """Return list of (marker_id:int, u:int, v:int, corners:ndarray)."""
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        if self._new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._params)

        # Keep an annotated copy for optional debugging.
        if self._save_debug:
            annotated = color_image.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            self._last_annotated = annotated

        results = []
        if ids is None:
            return results
        h, w = color_image.shape[:2]
        for marker_id, quad in zip(ids.flatten(), corners):
            pts = quad.reshape(-1, 2)
            u = int(np.clip(int(pts[:, 0].mean()), 0, w - 1))
            v = int(np.clip(int(pts[:, 1].mean()), 0, h - 1))
            results.append((int(marker_id), u, v, pts))
        return results

    def _publish_id_pixel(self, header, marker_id, u, v):
        self.id_pub.publish(Int32(data=int(marker_id)))
        px = PointStamped()
        px.header = header
        px.point.x = float(u)
        px.point.y = float(v)
        px.point.z = 0.0
        self.pixel_pub.publish(px)

    def _on_color_only(self, color_msg):
        if self.mode == "auto" and self._depth_seen:
            return
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge (color) failed: {exc}")
            return
        self._frames_seen += 1
        for marker_id, u, v, _ in self._detect(color):
            self._detections += 1
            self._publish_id_pixel(color_msg.header, marker_id, u, v)
            self.get_logger().info(
                f"Marker ID: {marker_id} | pixel (u, v) = ({u}, {v})  [2D]")

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
        for marker_id, u, v, _ in self._detect(color):
            self._detections += 1
            self._publish_id_pixel(color_msg.header, marker_id, u, v)

            if self._intrinsics is None:
                self.get_logger().info(
                    f"Marker ID: {marker_id} | pixel ({u}, {v}) — awaiting"
                    " camera_info")
                continue

            raw = float(depth[v, u])
            z = raw / 1000.0 if depth.dtype == np.uint16 else raw
            if z <= 0.0:
                self.get_logger().info(
                    f"Marker ID: {marker_id} | pixel ({u}, {v}) — no valid depth")
                continue

            fx, fy, cx, cy = self._intrinsics
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            pt = PointStamped()
            pt.header = color_msg.header
            pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
            self.point_pub.publish(pt)
            self.get_logger().info(
                f"Marker ID: {marker_id} | X: {x:+.3f}  Y: {y:+.3f}  "
                f"Z: {z:+.3f} m  [3D]")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCoordinatesNode()
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
