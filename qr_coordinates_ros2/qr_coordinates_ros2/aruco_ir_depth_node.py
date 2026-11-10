#!/usr/bin/env python3
"""
aruco_ir_depth_node.py
----------------------
ROS 2 Humble node that detects an ArUco marker in the Intel RealSense D435's
**infrared (IR) stream** and uses the **aligned depth** to deproject the
marker's center into real-world coordinates.

Key properties
==============
  * IR only for detection (no RGB). The left IR imager (infra1) is the native
    reference frame for depth, so depth aligns to it cleanly, and IR draws less
    power than RGB.
  * Deprojection (pinhole model from camera_info):
        Z = depth(u, v)                      # forward distance, meters
        X = (u - cx) * Z / fx                # right = +X
        Y = -(v - cy) * Z / fy               # UP = +Y  (Cartesian, Y flipped)
  * Origin = the camera's optical axis. When the marker is centered in the
    image (camera pointed straight at it), X and Y read ~ (0, 0) — exactly like
    the origin of a Cartesian plane. No manual origin calibration.
  * Continuously prints X and Y in **millimeters** (Z too).

Publish/subscribe (built for a later Pi -> Arduino bridge)
==========================================================
Subscribes (time-synchronized):
  <ir_topic>            sensor_msgs/Image       infra1 (IR) image
  <depth_topic>         sensor_msgs/Image       depth in infra1 frame (16UC1 mm)
  <camera_info_topic>   sensor_msgs/CameraInfo  infra1 intrinsics

Publishes:
  ~/marker/id        std_msgs/Int32              detected marker ID
  ~/marker/point     geometry_msgs/PointStamped  (x, y, z) in METERS, Cartesian
                                                 (+X right, +Y up, +Z forward)
  ~/marker/xy_mm_live geometry_msgs/Point        LIVE (x, y, z) in MILLIMETERS
                                                 for ANY distance — monitoring
  ~/marker/xy_mm     geometry_msgs/Point         MOTOR snapshot (x, y, z) in mm,
                                                 ONLY when Z <= max_z and two
                                                 frames agree — the stepper
                                                 captures its one-shot from here

All output coordinates are smoothed by a Temporal Moving Average (ma_window
frames) before being printed/published/gated.

A future Arduino link is just another subscriber: a small node (micro-ROS,
rosserial, or a pyserial script) subscribes to ~/marker/xy_mm and forwards the
values over USB serial. No change needed here.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Int32, String, Bool

# realsense2_camera publishes images with sensor-data QoS (BEST_EFFORT). A
# default (RELIABLE) subscriber silently receives nothing, which looks like
# "no frames" even though `ros2 topic hz` (auto-matches QoS) shows 30 Hz.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _get_aruco_dictionary(name):
    name = name.upper()
    const = getattr(cv2.aruco, name, None)
    if const is None:
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(const)
    return cv2.aruco.Dictionary_get(const)


class ArucoIrDepthNode(Node):
    def __init__(self):
        super().__init__("aruco_ir_depth_node")

        # ---- Parameters ----
        self.declare_parameter("aruco_dictionary", "DICT_4X4_250")
        # Only act on this marker ID; all other detections are ignored (noise /
        # wrong markers never reach the Arduino). Set to -1 to accept any ID.
        self.declare_parameter("target_id", 1)
        self.declare_parameter(
            "ir_topic", "/camera/camera/infra1/image_rect_raw")
        self.declare_parameter(
            "depth_topic",
            # Plain depth is already in the left-IR (infra1) frame on the D435,
            # so deprojecting with infra1 intrinsics is correct. The
            # aligned_depth_to_infra1 topic is not produced when color is off.
            "/camera/camera/depth/image_rect_raw")
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/infra1/camera_info")
        self.declare_parameter("sync_slop", 0.05)
        # How many times per second to PRINT the latest X/Y to the console.
        self.declare_parameter("print_rate_hz", 30.0)
        # Half-size of the depth sampling window (pixels). Depth is taken as the
        # median of valid pixels in a (2*w+1)^2 patch around the marker center.
        self.declare_parameter("depth_window", 20)
        # If True, keep publishing the last good reading when a frame has no
        # valid depth (so the stream does not stall on depth holes).
        # (Retained for compatibility; output no longer persists after the
        # marker is lost — see the found_target handling below.)
        self.declare_parameter("hold_last", True)
        # Smoothing: exponential moving average on X/Y/Z. 0 = no smoothing
        # (raw), 1 = frozen. 0.3 is a good balance (responsive but steady).
        self.declare_parameter("smoothing", 0.1)
        # If the marker is detected beyond +/- this many mm on either axis, the
        # reading is REJECTED (no snapshot, no motion) and we WAIT for a valid,
        # in-range reading. Slide is 150 mm (+/-75); 70 keeps a safety margin.
        # (<= 0 would disable the limit; 70 = reject anything over +/-70 mm.)
        self.declare_parameter("max_offset_mm", 70.0)
        # Fixed fallback distance (m) used to scale pixel->mm when measured
        # depth is unavailable. Camera-to-marker distance is < 25cm; 0.20m is a
        # reasonable mid value. Z only affects X/Y scale, not the centred point.
        self.declare_parameter("assumed_z", 0.20)
        # Require the target marker to be seen this many consecutive frames
        # before publishing, to reject single-frame false detections.
        self.declare_parameter("confirm_frames", 1)
        # Two consecutive frames must agree within this (mm) on BOTH X and Y to
        # confirm a stable target (the "snapshot"). Rejects noise.
        self.declare_parameter("agree_tol_mm", 0.1)
        # Z gate: only act when the marker is within this distance (m). < 0.25 m.
        # NOTE: this gates only the MOTOR path (snapshot/agreement). Live
        # readings are still deprojected, filtered, printed and published for
        # ALL distances so you can see the data even when Z > max_z.
        self.declare_parameter("max_z", 0.25)
        # Temporal Moving Average window (frames) applied to the output X/Y/Z.
        # The last N valid readings are averaged -> smoother, less noisy output.
        # 1 = disabled (raw). 5 is a good start at ~30 Hz (~0.17 s of data).
        self.declare_parameter("ma_window", 5)

        dict_name = str(self.get_parameter("aruco_dictionary").value)
        self._target_id = int(self.get_parameter("target_id").value)
        ir_topic = self.get_parameter("ir_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        # sync_slop kept as a declared param for launch compatibility (unused
        # now that we pair IR with the latest cached depth frame).

        self.bridge = CvBridge()

        # ArUco detector (new OpenCV 4.7+ and old APIs)
        self._aruco_dict = _get_aruco_dictionary(dict_name)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, params)
            self._new_api = True
        else:
            self._params = cv2.aruco.DetectorParameters_create()
            self._new_api = False

        self._intrinsics = None  # (fx, fy, cx, cy)
        self._frames_seen = 0      # IR frames processed
        self._depth_rx = 0         # depth frames received
        self._detections = 0
        self._latest_depth = None  # most recent depth cv image
        self._depth_window = int(self.get_parameter("depth_window").value)
        self._hold_last = bool(self.get_parameter("hold_last").value)
        self._smoothing = float(self.get_parameter("smoothing").value)
        self._max_offset_mm = float(self.get_parameter("max_offset_mm").value)
        self._assumed_z = float(self.get_parameter("assumed_z").value)
        self._confirm_frames = int(self.get_parameter("confirm_frames").value)
        self._agree_tol_mm = float(self.get_parameter("agree_tol_mm").value)
        self._max_z = float(self.get_parameter("max_z").value)
        self._ma_window = max(1, int(self.get_parameter("ma_window").value))
        # Rolling buffers for the Temporal Moving Average filter (meters).
        self._ma_x = deque(maxlen=self._ma_window)
        self._ma_y = deque(maxlen=self._ma_window)
        self._ma_z = deque(maxlen=self._ma_window)
        self._prev_xy = None      # previous frame's (x, y) for agreement check
        # Smoothing + confirmation state.
        self._ema = None          # (x, y, z) smoothed, meters
        self._seen_streak = 0     # consecutive frames the target was detected
        # Latest marker result for the throttled printer:
        # (marker_id, x_mm, y_mm, z_mm) or None if nothing seen recently.
        self._latest_result = None

        # ---- Publishers ----
        self.id_pub = self.create_publisher(Int32, "~/marker/id", 10)
        self.point_pub = self.create_publisher(
            PointStamped, "~/marker/point", 10)
        # MOTOR snapshot topic: published ONLY for Z-gated, agreement-confirmed
        # readings. The stepper captures its one-shot target from this topic.
        self.xy_mm_pub = self.create_publisher(Point, "~/marker/xy_mm", 10)
        # LIVE topic: published for EVERY valid-depth reading at ANY distance
        # (TMA-filtered). Lets you monitor the data even when Z > max_z, without
        # feeding the motor path. Mirrors ~/marker/xy_mm's (x, y, z in mm).
        self.xy_mm_live_pub = self.create_publisher(
            Point, "~/marker/xy_mm_live", 10)
        # Combined human-readable topic: "id=7 x=12.3 y=-4.8 z=512.0 (mm)".
        # Convenient so `ros2 topic echo` shows id + X/Y/Z together.
        self.info_pub = self.create_publisher(String, "~/marker/info", 10)
        # Human-readable STATUS messages (e.g. "move the robot closer"). These
        # are both PUBLISHED here and printed to stdout, so they appear even
        # when stderr is discarded (run.sh launches with 2>/dev/null, and ROS
        # logger output goes to stderr).
        self.status_pub = self.create_publisher(String, "~/marker/status", 10)
        # Presence flag, published EVERY IR frame: True = target seen (with
        # valid depth) this frame, False = not seen. Lets the stepper stop
        # IMMEDIATELY when the marker disappears (no timeout needed).
        self.present_pub = self.create_publisher(Bool, "~/marker/present", 10)

        # camera_info uses default (RELIABLE) QoS in realsense2_camera.
        self.create_subscription(CameraInfo, info_topic, self._on_info, 10)

        # Instead of a time-synchronizer (which silently matched nothing here),
        # subscribe independently with sensor-data (BEST_EFFORT) QoS and pair
        # each IR frame with the most-recent depth frame. The D435 IR and
        # aligned-depth share the same imager/timing, so the latest depth is an
        # excellent match for the current IR frame.
        self.create_subscription(
            Image, depth_topic, self._on_depth, SENSOR_QOS)
        self.create_subscription(
            Image, ir_topic, self._on_ir, SENSOR_QOS)

        self.create_timer(3.0, self._heartbeat)

        # Throttled printer: detection runs at the IR frame rate (~30 Hz) and
        # keeps publishing on topics every frame, but we only PRINT the latest
        # X/Y at print_rate_hz (default 30 Hz, matching the camera frame rate).
        print_hz = float(self.get_parameter("print_rate_hz").value)
        if print_hz > 0.0:
            self.create_timer(1.0 / print_hz, self._print_latest)

        target_note = ("ANY" if self._target_id < 0 else str(self._target_id))
        self.get_logger().info(
            "aruco_ir_depth_node started.\n"
            f"  dictionary:  {dict_name}\n"
            f"  target_id:   {target_note} (other IDs ignored)\n"
            f"  ir:          {ir_topic}\n"
            f"  depth:       {depth_topic}\n"
            f"  camera_info: {info_topic}\n"
            "Origin = camera optical axis; marker centered => (0, 0).\n"
            "Convention: +X right, +Y up (Cartesian). Units: millimeters."
        )

    def _on_info(self, msg: CameraInfo):
        k = msg.k
        if k[0] != 0.0 and k[4] != 0.0:
            self._intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def _heartbeat(self):
        if self._frames_seen == 0 and self._depth_rx == 0:
            self.get_logger().warn(
                "No IR or depth frames yet — is the camera streaming infra1 "
                "and aligned depth? (needs enable_infra1 + align_depth)")
            return
        # Diagnostic only (stream health). Not the marker ID or position.
        self.get_logger().debug(
            f"streams ok: {self._frames_seen} IR, {self._depth_rx} depth "
            f"frames received.")

    def _detect(self, ir_gray):
        """Detect ArUco in a single-channel IR image. Returns list of
        (id, u, v)."""
        if self._new_api:
            corners, ids, _ = self._detector.detectMarkers(ir_gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                ir_gray, self._aruco_dict, parameters=self._params)
        out = []
        if ids is None:
            return out
        h, w = ir_gray.shape[:2]
        for marker_id, quad in zip(ids.flatten(), corners):
            pts = quad.reshape(-1, 2)
            u = int(np.clip(int(pts[:, 0].mean()), 0, w - 1))
            v = int(np.clip(int(pts[:, 1].mean()), 0, h - 1))
            out.append((int(marker_id), u, v))
        return out

    def _on_depth(self, depth_msg):
        """Cache the most recent depth frame."""
        try:
            self._latest_depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
            self._depth_rx += 1
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge (depth) failed: {exc}")

    def _on_ir(self, ir_msg):
        """On each IR frame: detect markers, pair with the latest depth."""
        try:
            # IR is mono8 (sometimes mono16); ArUco wants 8-bit single channel.
            ir = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding="passthrough")
            if ir.dtype != np.uint8:
                ir = cv2.convertScaleAbs(ir)
            if ir.ndim == 3:
                ir = cv2.cvtColor(ir, cv2.COLOR_BGR2GRAY)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge (IR) failed: {exc}")
            return

        self._frames_seen += 1

        # Depth is REQUIRED now (no assumed-Z fallback). Wait for both a depth
        # frame and the camera intrinsics before doing anything. Stay SILENT
        # until then — do NOT publish present=false every frame (that floods
        # `ros2 topic echo` with endless "---" separators before the camera
        # has anything to report).
        depth = self._latest_depth
        if depth is None or self._intrinsics is None:
            return
        fx, fy, cx, cy = self._intrinsics

        dh, dw = depth.shape[:2]
        found_target = False
        published_valid = False     # a usable X/Y was published this frame
        for marker_id, u, v in self._detect(ir):
            # Only act on the target marker; ignore any other ID (noise /
            # wrong markers never reach the Arduino). target_id = -1 = any.
            if self._target_id >= 0 and marker_id != self._target_id:
                continue
            found_target = True
            # Clamp to the depth image bounds.
            du = min(u, dw - 1)
            dv = min(v, dh - 1)

            # Measure depth (median of valid pixels in a window around centre).
            z = 0.0
            if depth is not None:
                r = self._depth_window  # half-size
                y0, y1 = max(0, dv - r), min(dh, dv + r + 1)
                x0, x1 = max(0, du - r), min(dw, du + r + 1)
                patch = depth[y0:y1, x0:x1].astype(np.float32)
                valid = patch[patch > 0]
                if valid.size > 0:
                    raw = float(np.median(valid))
                    z = raw / 1000.0 if depth.dtype == np.uint16 else raw

            # Need a valid depth to deproject at all. No depth -> skip frame.
            if z <= 0.0:
                self._prev_xy = None
                continue

            # Deproject. Cartesian: +X right, +Y UP (flip image-down v).
            x = (u - cx) * z / fx          # meters, right = +
            y = -(v - cy) * z / fy         # meters, up = +  (negated)

            # --- TEMPORAL MOVING AVERAGE on the output coordinates ---
            # Push this frame's raw (x, y, z) and use the mean of the last N.
            # Smooths camera noise before anything is printed/published/gated.
            self._ma_x.append(x)
            self._ma_y.append(y)
            self._ma_z.append(z)
            x = float(np.mean(self._ma_x))
            y = float(np.mean(self._ma_y))
            z = float(np.mean(self._ma_z))

            # --- LIVE OUTPUT (ANY distance) ---
            # Always print/publish the filtered reading so you can see the data
            # even when Z > max_z. This path NEVER touches the motor snapshot.
            x_mm, y_mm, z_mm = x * 1000.0, y * 1000.0, z * 1000.0
            found_target = True  # a usable live reading exists this frame

            self.id_pub.publish(Int32(data=marker_id))
            pt = PointStamped()
            pt.header = ir_msg.header
            pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
            self.point_pub.publish(pt)
            live = Point()
            live.x, live.y, live.z = float(x_mm), float(y_mm), float(z_mm)
            self.xy_mm_live_pub.publish(live)
            # Feed the throttled console printer (shows live X/Y/Z at any Z).
            self._latest_result = (marker_id, x_mm, y_mm, z_mm)

            # --- Z GATE for the MOTOR path only ---
            # Beyond max_z: keep streaming live data (done above) but do NOT
            # snapshot or publish ~/marker/xy_mm, and reset the agreement.
            # Also PUBLISH + PRINT a throttled prompt to move the robot closer.
            if z > self._max_z:
                self._prev_xy = None
                self._far_count = getattr(self, "_far_count", 0) + 1
                if self._far_count % 30 == 1:   # ~once/sec at 30 Hz
                    msg = (f"Marker is {z * 100:.1f} cm away (more than "
                           f"{self._max_z * 100:.0f} cm) — move the robot "
                           "CLOSER to align.")
                    # Publish on the status topic...
                    self.status_pub.publish(String(data=msg))
                    # ...and print to STDOUT so it shows in the main terminal
                    # even though run.sh discards stderr (where logger writes).
                    print(msg, flush=True)
                continue
            # In range again -> reset the throttle so the next far event prints.
            self._far_count = 0

            # Optional range limit (motor path). max_offset_mm <= 0 = no limit.
            if self._max_offset_mm > 0.0:
                m_m = self._max_offset_mm / 1000.0
                if abs(x) > m_m or abs(y) > m_m:
                    self._oor_count = getattr(self, "_oor_count", 0) + 1
                    if self._oor_count % 30 == 1:
                        self.get_logger().warn(
                            f"Marker out of range (|X|={x*1000:.0f} or "
                            f"|Y|={y*1000:.0f} mm > {self._max_offset_mm:.0f}) "
                            "— ignoring, not moving.")
                    self._prev_xy = None
                    continue

            # STABLE-FRAME CONFIRMATION (motor snapshot): require THIS frame's
            # (x,y) to agree with the PREVIOUS within agree_tol_mm. Only then is
            # it a confirmed snapshot the stepper may capture and move to.
            tol = self._agree_tol_mm / 1000.0
            prev = self._prev_xy
            self._prev_xy = (x, y)
            if prev is None or abs(x - prev[0]) > tol or abs(y - prev[1]) > tol:
                # Not yet two matching frames — wait for the next one.
                continue

            # Two frames agree -> confirmed target = average of the two.
            x = 0.5 * (x + prev[0])
            y = 0.5 * (y + prev[1])
            x_mm, y_mm = x * 1000.0, y * 1000.0

            self._detections += 1

            # Publish the MOTOR snapshot (stepper captures its one-shot here).
            xy = Point()
            xy.x, xy.y, xy.z = float(x_mm), float(y_mm), float(z_mm)
            self.xy_mm_pub.publish(xy)
            published_valid = True

        # Target not seen (no valid reading) this frame -> clear state.
        if not found_target:
            self._seen_streak = 0
            self._ema = None
            self._prev_xy = None       # restart agreement on re-acquire
            self._latest_result = None
            # Clear the moving-average buffers so a re-acquired marker starts
            # fresh rather than averaging across a gap.
            self._ma_x.clear()
            self._ma_y.clear()
            self._ma_z.clear()

        # Presence flag EVERY frame: True only when a Z-gated, agreement-
        # confirmed snapshot (~/marker/xy_mm) was published this frame. Live
        # readings beyond max_z do NOT set this, so the motor path stays gated.
        self.present_pub.publish(Bool(data=published_valid))

    def _print_latest(self):
        """Print the most recent marker X/Y at a fixed rate (default 10 Hz)."""
        res = self._latest_result
        if res is None:
            return
        marker_id, x_mm, y_mm, z_mm = res
        line = (f"ID {marker_id}  X: {x_mm:+7.1f} mm  Y: {y_mm:+7.1f} mm  "
                f"(Z: {z_mm:7.1f} mm)")
        # 1st terminal: print at the chosen rate.
        self.get_logger().info(line)
        # 2nd terminal: publish the same readable line so `ros2 topic echo
        # ~/marker/info` shows id + X/Y/Z together at the print rate.
        self.info_pub.publish(String(
            data=f"id={marker_id} x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} (mm)"))
        # Clear after emitting so a value is never repeated: each printed line
        # corresponds to a fresh detection. When the marker is removed the
        # detection loop stops setting _latest_result, so output goes silent.
        self._latest_result = None


def main(args=None):
    rclpy.init(args=args)
    node = ArucoIrDepthNode()
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
