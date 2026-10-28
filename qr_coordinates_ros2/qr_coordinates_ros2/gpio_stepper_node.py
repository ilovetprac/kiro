#!/usr/bin/env python3
"""
gpio_stepper_node.py
--------------------
Drive two stepper motors DIRECTLY from the Raspberry Pi GPIO (no Arduino) via
two STEP/DIR microstep drivers (TB6600-style), to align the AGV female
connector to an ArUco marker.

Replaces the Arduino sketch + serial bridge: this node subscribes to the marker
offset and generates STEP/DIR pulses on the Pi GPIO using pigpio (hardware-timed
waveforms via DMA — far smoother/steadier than bit-banging on a busy Pi).

Pipeline:
  aruco_ir_depth_node  --(/marker/xy_mm, mm)-->  THIS node  -->  GPIO STEP/DIR

Behaviour (matches the previous Arduino logic):
  * Move only when the target marker (ID 1) is detected & in range. The vision
    node only publishes when ID 1 is confirmed (2 frames) and within +/-70 mm,
    so receiving a message == "valid target present".
  * STOP IMMEDIATELY (hold position, no stepping) if no marker message arrives
    within lost_timeout_s.
  * Always chase the LATEST reading (real-time correction).
  * Two-speed: fast when far, fine when close; deadband = aligned.
  * +/- travel clamp from the power-on centre (no end-stops -> centre first).

Wiring (per driver, common-cathode):
  Pi GPIO STEP pin -> PUL+ , Pi GPIO DIR pin -> DIR+ , Pi GND -> PUL- and DIR-
  Driver VCC/GND -> 24 V supply ; A+/A-/B+/B- -> motor coils. (Pi never touches 24 V.)
  If 3.3 V doesn't reliably trigger the opto inputs, add a 3.3->5 V level shifter
  on PUL+/DIR+.

Default BCM pins (override via parameters):
  X: STEP=GPIO17 (pin 11), DIR=GPIO27 (pin 13)
  Y: STEP=GPIO22 (pin 15), DIR=GPIO23 (pin 16)
  Common GND: any Pi GND (e.g. pin 9) -> both drivers' PUL-/DIR-.

Requires pigpio:  sudo apt install pigpio python3-pigpio
Start the daemon:  sudo systemctl enable --now pigpiod   (or: sudo pigpiod)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

try:
    import pigpio
except ImportError:
    pigpio = None


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
        self.declare_parameter("deadband_mm", 1.0)
        self.declare_parameter("fine_zone_mm", 5.0)
        self.declare_parameter("fast_sps", 4000)   # steps/sec far from target
        self.declare_parameter("fine_sps", 800)    # steps/sec near target
        self.declare_parameter("max_travel_mm", 70.0)   # +/- from power-on centre
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("control_rate_hz", 100.0)  # control loop rate
        self.declare_parameter("lost_timeout_s", 0.5)

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
        rate = float(g("control_rate_hz").value)
        self.lost_timeout = float(g("lost_timeout_s").value)

        self.step_limit = int(self.max_travel * self.steps_per_mm)

        # ---- State ----
        self.latest = None        # (x_mm, y_mm) latest target
        self.last_rx = 0.0
        self.pos_x = 0            # steps from power-on centre
        self.pos_y = 0

        # ---- pigpio ----
        if pigpio is None:
            self.get_logger().error(
                "pigpio not installed. Run: sudo apt install pigpio "
                "python3-pigpio  && sudo systemctl enable --now pigpiod")
            self.pi = None
        else:
            self.pi = pigpio.pi()        # connect to pigpiod
            if not self.pi.connected:
                self.get_logger().error(
                    "Could not connect to pigpiod. Start it: "
                    "sudo systemctl enable --now pigpiod")
                self.pi = None
            else:
                for p in (self.xs, self.xd, self.ys, self.yd):
                    self.pi.set_mode(p, pigpio.OUTPUT)
                    self.pi.write(p, 0)

        self.create_subscription(Point, topic, self._on_marker, 10)
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._control)

        self.get_logger().info(
            "gpio_stepper_node started.\n"
            f"  topic:        {topic} (mm)\n"
            f"  X step/dir:   GPIO{self.xs}/{self.xd}\n"
            f"  Y step/dir:   GPIO{self.ys}/{self.yd}\n"
            f"  steps/mm:     {self.steps_per_mm}\n"
            f"  deadband:     {self.deadband} mm, travel +/-{self.max_travel} mm\n"
            f"  speeds:       fast {self.fast_sps} sps, fine {self.fine_sps} sps\n"
            "Centre the slides before use (power-on = 0,0). No end-stops.")

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_marker(self, msg: Point):
        # Valid target present (vision node gates ID 1 / 2-frame / +/-70mm).
        self.latest = (float(msg.x), float(msg.y))
        self.last_rx = self._now()

    def _pulse_axis(self, offset_mm, step_pin, dir_pin, pos_attr, invert):
        """Emit a short burst of steps toward cancelling offset_mm. Non-blocking
        enough: burst length is small so the control loop stays responsive and
        always re-chases the latest target."""
        if self.pi is None:
            return
        if abs(offset_mm) <= self.deadband:
            return  # within 1 mm -> hold (coils stay energised by the driver)

        fine = abs(offset_mm) <= self.fine_zone
        sps = self.fine_sps if fine else self.fast_sps

        # Steps to move this control tick = sps / control_rate, capped by the
        # steps actually needed to cancel the offset.
        want = int(abs(offset_mm) * self.steps_per_mm)
        per_tick = max(1, int(sps / 100))      # ~ control loop period
        n = min(want, per_tick)
        if n <= 0:
            return

        # Direction: move to REDUCE a positive offset; invert for wiring.
        positive = offset_mm < 0            # negative offset -> step "forward"
        if invert:
            positive = not positive

        pos = getattr(self, pos_attr)
        # Clamp to +/- travel limit (no end-stops).
        if positive and pos + n > self.step_limit:
            n = self.step_limit - pos
        if (not positive) and pos - n < -self.step_limit:
            n = pos + self.step_limit
        if n <= 0:
            return

        self.pi.write(dir_pin, 1 if positive else 0)

        # Build a hardware-timed pulse train: n pulses at the chosen rate.
        half_us = max(2, int(1_000_000 / (2 * sps)))   # half-period, >=2us
        self.pi.wave_clear()
        pulses = []
        pulses.append(pigpio.pulse(1 << step_pin, 0, half_us))
        pulses.append(pigpio.pulse(0, 1 << step_pin, half_us))
        self.pi.wave_add_generic(pulses)
        wid = self.pi.wave_create()
        if wid >= 0:
            # Send the single-pulse wave n times (chain) for n steps.
            self.pi.wave_chain([255, 0, wid, 255, 1, n & 0xFF, (n >> 8) & 0xFF])
            while self.pi.wave_tx_busy():
                pass
            self.pi.wave_delete(wid)

        setattr(self, pos_attr, pos + (n if positive else -n))

    def _control(self):
        # Marker lost -> stop (hold). Do nothing (no stepping).
        if self.latest is None or (self._now() - self.last_rx) > self.lost_timeout:
            return
        x, y = self.latest
        self._pulse_axis(x, self.xs, self.xd, "pos_x", self.invert_x)
        self._pulse_axis(y, self.ys, self.yd, "pos_y", self.invert_y)

    def destroy_node(self):
        if self.pi is not None:
            try:
                for p in (self.xs, self.xd, self.ys, self.yd):
                    self.pi.write(p, 0)
                self.pi.stop()
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
