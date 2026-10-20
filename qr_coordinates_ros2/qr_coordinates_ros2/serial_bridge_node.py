#!/usr/bin/env python3
"""
serial_bridge_node.py
---------------------
Bridges the ArUco marker X/Y offset from ROS 2 to the Arduino over USB serial,
for the AGV female-connector alignment (2 stepper screw slides).

Subscribes:
  <marker_topic>   geometry_msgs/Point   marker offset (x, y, z) in MILLIMETERS
                                          (default /aruco_ir_depth_node/marker/xy_mm)

Writes to the Arduino (default /dev/ttyUSB0 @ 115200), one line per update:
    "X,Y\n"      e.g. "12.3,-4.8"      (millimeters; Z is ignored by design)

The Arduino (agv_xy_align.ino) drives the X/Y steppers to bring X,Y toward 0.

Parameters:
  marker_topic   source topic (geometry_msgs/Point, mm)   default xy_mm topic
  serial_port    default "/dev/ttyUSB0"
  baud           default 115200
  send_rate_hz   how often to send to the Arduino          default 20.0
  deadband_mm    don't bother sending tiny offsets         default 1.0

Run:
  ros2 run qr_coordinates_ros2 serial_bridge_node
  ros2 run qr_coordinates_ros2 serial_bridge_node --ros-args -p serial_port:=/dev/ttyUSB0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

try:
    import serial  # pyserial
except ImportError:
    serial = None


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__("serial_bridge_node")

        self.declare_parameter(
            "marker_topic", "/aruco_ir_depth_node/marker/xy_mm")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("deadband_mm", 1.0)

        topic = self.get_parameter("marker_topic").value
        self.port = self.get_parameter("serial_port").value
        self.baud = int(self.get_parameter("baud").value)
        rate = float(self.get_parameter("send_rate_hz").value)
        self.deadband = float(self.get_parameter("deadband_mm").value)

        self._latest = None   # (x_mm, y_mm)
        self._ser = None

        if serial is None:
            self.get_logger().error(
                "pyserial not installed. Install with: pip3 install pyserial "
                "(or: sudo apt install python3-serial)")
        else:
            self._open_serial()

        self.create_subscription(Point, topic, self._on_marker, 10)

        if rate > 0.0:
            self.create_timer(1.0 / rate, self._send)

        self.get_logger().info(
            "serial_bridge_node started.\n"
            f"  subscribing: {topic} (mm)\n"
            f"  serial:      {self.port} @ {self.baud}\n"
            f"  send rate:   {rate} Hz, deadband {self.deadband} mm")

    def _open_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
        except Exception as exc:  # noqa: BLE001
            self._ser = None
            self.get_logger().error(
                f"Could not open {self.port}: {exc}. Is the Arduino plugged in? "
                "Check the port with: ls /dev/ttyUSB* /dev/ttyACM* and that your "
                "user is in the 'dialout' group.")

    def _on_marker(self, msg: Point):
        # msg.x, msg.y are millimeters (z ignored by design).
        self._latest = (float(msg.x), float(msg.y))

    def _send(self):
        if self._latest is None:
            return
        if self._ser is None:
            # Try to reopen (e.g. Arduino was plugged in after start).
            self._open_serial()
            if self._ser is None:
                return

        x, y = self._latest
        # Optional: skip sending if within deadband on both axes.
        if abs(x) < self.deadband and abs(y) < self.deadband:
            line = "0,0\n"
        else:
            line = f"{x:.1f},{y:.1f}\n"

        try:
            self._ser.write(line.encode("ascii"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Serial write failed: {exc}; will reopen.")
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._ser is not None:
            try:
                node._ser.close()
            except Exception:  # noqa: BLE001
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
