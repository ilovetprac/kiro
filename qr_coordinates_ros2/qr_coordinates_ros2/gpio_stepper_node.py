#!/usr/bin/env python3
"""
gpio_stepper_node.py
--------------------
Drive two stepper motors DIRECTLY from the Raspberry Pi GPIO (no Arduino) via
two STEP/DIR microstep drivers (TB6600-style), to align the AGV female
connector to an ArUco marker.

Uses lgpio (kernel gpiochip interface) — the supported GPIO library on
Ubuntu 22.04 for the Pi. No daemon required (unlike pigpio).

Pipeline:
  aruco_ir_depth_node  --(/marker/xy_mm, mm)-->  THIS node  -->  GPIO STEP/DIR

Behaviour:
  * Move only when the target marker (ID 1) is detected & in range. The vision
    node only publishes when ID 1 is confirmed (2 frames) and within +/-70 mm,
    so receiving a message == "valid target present".
  * STOP IMMEDIATELY when the vision node reports no marker (~/marker/present
    = False). Otherwise act on the LATEST reading only (no timeout).
  * Always chase the LATEST reading (real-time correction).
  * Two-speed: fast when far, fine when close. NO lock/latch — it corrects
    continuously in real time; the small deadband only prevents jitter.
  * +/- travel clamp from the power-on centre (no end-stops -> centre first).

Wiring (per driver, common-cathode):
  Pi GPIO STEP pin -> PUL+ , Pi GPIO DIR pin -> DIR+ , Pi GND -> PUL- and DIR-
  Driver VCC/GND -> 24 V ; A+/A-/B+/B- -> motor coils. (Pi never touches 24 V.)
  Motor colours (YUN DUAN label): A+ black, A- green, B+ red, B- blue.

Default BCM pins (override via parameters):
  X: STEP=GPIO17 (pin 11), DIR=GPIO27 (pin 13)
  Y: STEP=GPIO22 (pin 15), DIR=GPIO23 (pin 16)
  Common GND: any Pi GND -> both drivers' PUL-/DIR-.

Install:  sudo apt install python3-lgpio
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

try:
    import lgpio
except ImportError:
    lgpio = None


class GpioStepperNode(Node):
    def __init__(self):
        super().__init__("gpio_stepper_node")

        # ---- Pins (BCM numbering) ----
        self.declare_parameter("x_step_pin", 17)
        self.declare_parameter("x_dir_pin", 27)
        self.declare_parameter("y_step_pin", 22)
        self.declare_parameter("y_dir_pin", 23)

        # ---- Mechanics / control ----
        self.declare_parameter("marker_topic",
                               "/aruco_ir_depth_node/marker/xy_mm")
        self.declare_parameter("steps_per_mm", 1600.0)   # 1/8 microstep, 1mm lead
        # No alignment "lock"/latch — the node ALWAYS corrects in real time.
        # The deadband is only a tiny no-jitter zone: inside it the motor holds
        # (camera noise is ~sub-mm, so a 0 deadband makes the motors buzz
        # forever). 0.5 mm gives continuous fine correction without jitter.
        self.declare_parameter("deadband_mm", 0.5)
        self.declare_parameter("fine_zone_mm", 5.0)
        self.declare_parameter("fast_sps", 4000)   # steps/sec far from target
        self.declare_parameter("fine_sps", 800)    # steps/sec near target
        self.declare_parameter("max_travel_mm", 70.0)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("control_rate_hz", 100.0)



        g = self.get_parameter
        self.xs = int(g("x_step_pin").value); self.xd = int(g("x_dir_pin").value)
        self.ys = int(g("y_step_pin").value); self.yd = int(g("y_dir_pin").value)
        topic = g("marker_topic").value
        self.steps_per_mm = float(g("steps_per_mm").value)
        self.deadband = float(g("deadband_mm").value)
        self.fine_zone = float(g("fine_zone_mm").value)
        self.fast_sps = int(g("fast_sps").value)
        self.fine_sps = int(g("fine_sps").value)
        self.max_travel = float(g("max_travel_mm").value)
        self.invert_x = bool(g("invert_x").value)
        self.invert_y = bool(g("invert_y").value)
        self.rate = float(g("control_rate_hz").value)

        self.step_limit = int(self.max_travel * self.steps_per_mm)

        # ---- State ----
        self.latest = None       # (x_mm, y_mm) latest target
        self.present = False     # marker present THIS instant (from vision node)
        self.pos_x = 0
        self.pos_y = 0

        # ---- lgpio ----
        self.h = None
        if lgpio is None:
            self.get_logger().error(
                "lgpio not installed. Run: sudo apt install python3-lgpio")
        else:
            try:
                self.h = lgpio.gpiochip_open(0)   # /dev/gpiochip0
                for p in (self.xs, self.xd, self.ys, self.yd):
                    lgpio.gpio_claim_output(self.h, p, 0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"lgpio init failed: {exc}")
                self.h = None

        self.create_subscription(Point, topic, self._on_marker, 10)
        # Presence flag (published every frame by the vision node): stop
        # IMMEDIATELY when False (no marker). Derive its topic from marker_topic.
        present_topic = topic.rsplit("/", 1)[0] + "/present"
        self.create_subscription(Bool, present_topic, self._on_present, 10)
        if self.rate > 0.0:
            self.create_timer(1.0 / self.rate, self._control)

        self.get_logger().info(
            "gpio_stepper_node started (lgpio).\n"
            f"  topic:      {topic} (mm)\n"
            f"  X step/dir: GPIO{self.xs}/{self.xd}\n"
            f"  Y step/dir: GPIO{self.ys}/{self.yd}\n"
            f"  steps/mm:   {self.steps_per_mm}, deadband {self.deadband} mm, "
            f"travel +/-{self.max_travel} mm\n"
            f"  speeds:     fast {self.fast_sps} sps, fine {self.fine_sps} sps\n"
            "Centre the slides before use (power-on = 0,0). No end-stops.")

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_marker(self, msg: Point):
        self.latest = (float(msg.x), float(msg.y))

    def _on_present(self, msg: Bool):
        self.present = bool(msg.data)
        if not self.present:
            # Marker gone -> stop acting on the last reading immediately.
            self.latest = None

    def _step_burst(self, step_pin, n, sps):
        """Emit n step pulses on step_pin at ~sps steps/sec (blocking, short)."""
        if self.h is None or n <= 0:
            return
        half = max(0.00005, 1.0 / (2.0 * sps))   # half-period seconds (>=50us)
        for _ in range(n):
            lgpio.gpio_write(self.h, step_pin, 1)
            time.sleep(half)
            lgpio.gpio_write(self.h, step_pin, 0)
            time.sleep(half)

    def _pulse_axis(self, offset_mm, step_pin, dir_pin, pos_attr, invert):
        if self.h is None:
            return
        if abs(offset_mm) <= self.deadband:
            return  # within deadband -> hold

        fine = abs(offset_mm) <= self.fine_zone
        sps = self.fine_sps if fine else self.fast_sps

        # Steps this control tick: cap by rate and by steps needed.
        want = int(abs(offset_mm) * self.steps_per_mm)
        per_tick = max(1, int(sps / self.rate))
        n = min(want, per_tick)
        if n <= 0:
            return

        positive = offset_mm < 0           # negative offset -> step "forward"
        if invert:
            positive = not positive

        pos = getattr(self, pos_attr)
        if positive and pos + n > self.step_limit:
            n = self.step_limit - pos
        if (not positive) and pos - n < -self.step_limit:
            n = pos + self.step_limit
        if n <= 0:
            return

        lgpio.gpio_write(self.h, dir_pin, 1 if positive else 0)
        self._step_burst(step_pin, n, sps)
        setattr(self, pos_attr, pos + (n if positive else -n))

    def _control(self):
        # No marker present -> stop IMMEDIATELY (no timeout). Otherwise act on
        # the LATEST reading only.
        if not self.present or self.latest is None:
            return
        x, y = self.latest
        self._pulse_axis(x, self.xs, self.xd, "pos_x", self.invert_x)
        self._pulse_axis(y, self.ys, self.yd, "pos_y", self.invert_y)

    def destroy_node(self):
        if self.h is not None:
            try:
                for p in (self.xs, self.xd, self.ys, self.yd):
                    lgpio.gpio_write(self.h, p, 0)
                lgpio.gpiochip_close(self.h)
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpioStepperNode()
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
