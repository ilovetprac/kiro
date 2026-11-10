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

Behaviour (ONE-SHOT, no loop):
  * Wait for the vision node to publish ONE confirmed target. The vision node
    only publishes after two consecutive frames agree within 0.1 mm AND the
    marker is within the Z gate (< 25 cm), so the first message is a solid,
    stable "snapshot".
  * Capture that X/Y as a fixed goal, move each axis to it open-loop, then
    STOP. Once the move is done the PROGRAM EXITS automatically. There is NO
    loop and NO re-arm.
  * It does NOT stop or re-arm if the marker is later lost: the captured
    snapshot is all it needs; losing the marker mid-move must not abort it.
  * Two-speed with accel/decel ramp: fast when far, fine when close.
  * +/- travel clamp from the power-on centre (no end-stops -> centre first).

Exit:
  * The program exits on its own once the move completes. Pressing Ctrl+C
    before then also exits (slides stay where they are; no return-to-origin).

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
        self.declare_parameter("fast_sps", 1600)   # steps/sec far from target
        self.declare_parameter("fine_sps", 1600)    # steps/sec near target
        self.declare_parameter("max_travel_mm", 70.0)
        self.declare_parameter("invert_x", True)
        self.declare_parameter("invert_y", True)
        self.declare_parameter("control_rate_hz", 100.0)
        # Seconds to ramp from a standstill up to full target speed (and the
        # same rate to ramp back down). Smooths starts/stops, less rough/noisy.
        self.declare_parameter("accel_time_s", 1.0)



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
        self.accel_time = float(g("accel_time_s").value)

        self.step_limit = int(self.max_travel * self.steps_per_mm)

        # ---- State ----
        # One-shot snapshot-and-move state machine (no loop, no re-arm):
        #   ARMED  -> waiting to capture the ONE confirmed target
        #   MOVING -> driving to the captured goal (open loop)
        #   DONE   -> reached the goal; the program then SHUTS DOWN
        self.latest = None       # (x_mm, y_mm) first confirmed reading
        self.state = "ARMED"
        self._finished = False   # set True when the move is done -> main() exits
        self.goal_x_mm = 0.0     # captured target offset (mm) at snapshot time
        self.goal_y_mm = 0.0
        self.goal_pos_x = 0      # captured goal in absolute steps
        self.goal_pos_y = 0
        # Current ramped speed per axis (sps) for smooth accel/decel.
        self.cur_sps_x = 0.0
        self.cur_sps_y = 0.0
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
        # One-shot: we ignore ~/marker/present entirely. We only need the ONE
        # confirmed snapshot; losing the marker afterwards must NOT abort the
        # move or re-arm. No presence subscription.
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
        # Capture only the FIRST confirmed reading; ignore the rest. The vision
        # node already gated on Z<25cm + two frames agreeing within 0.1mm.
        if self.state == "ARMED" and self.latest is None:
            self.latest = (float(msg.x), float(msg.y))

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

    def _control(self):
        # ARMED: capture the first confirmed target as a fixed goal, then move.
        if self.state == "ARMED":
            if self.latest is None:
                return  # wait for a confirmed reading
            gx, gy = self.latest
            # Goal in absolute steps = current position + offset-to-cancel.
            # Moving to cancel the offset: dir sign handled in _move_axis.
            self.goal_x_mm, self.goal_y_mm = gx, gy
            self.goal_pos_x = self.pos_x + self._offset_to_steps(gx, self.invert_x)
            self.goal_pos_y = self.pos_y + self._offset_to_steps(gy, self.invert_y)
            # Clamp goal to travel limits.
            self.goal_pos_x = max(-self.step_limit,
                                  min(self.step_limit, self.goal_pos_x))
            self.goal_pos_y = max(-self.step_limit,
                                  min(self.step_limit, self.goal_pos_y))
            self.state = "MOVING"
            self.get_logger().info(
                f"Captured target X={gx:+.1f} Y={gy:+.1f} mm -> moving.")
            return

        # MOVING: drive each axis to its captured goal (open loop), then stop.
        if self.state == "MOVING":
            dx = self._move_to(self.xs, self.xd, "pos_x", "cur_sps_x",
                               self.goal_pos_x)
            dy = self._move_to(self.ys, self.yd, "pos_y", "cur_sps_y",
                               self.goal_pos_y)
            if not dx and not dy:
                self.state = "DONE"
                self.cur_sps_x = 0.0
                self.cur_sps_y = 0.0
                self.get_logger().info(
                    "Reached target -> STOPPED. One-shot complete; exiting.")
                # Signal main() to shut the program down now that the move is
                # finished and the motors have stopped.
                self._finished = True
            return

        # DONE: move complete -> main() shuts the program down.

    def _offset_to_steps(self, offset_mm, invert):
        """Steps to cancel offset_mm (signed). +offset -> move to reduce it."""
        steps = int(round(abs(offset_mm) * self.steps_per_mm))
        positive = offset_mm < 0
        if invert:
            positive = not positive
        return steps if positive else -steps

    def _move_to(self, step_pin, dir_pin, pos_attr, sps_attr, goal_pos):
        """Step toward goal_pos (absolute steps) with accel/decel ramp.
        Returns True if still moving, False if arrived."""
        if self.h is None:
            return False
        pos = getattr(self, pos_attr)
        remaining = goal_pos - pos
        if remaining == 0:
            setattr(self, sps_attr, 0.0)
            return False

        # Ramp speed toward fast_sps; ease down near the end (decel).
        cur = getattr(self, sps_attr)
        max_sps = float(self.fast_sps)
        max_delta = (max_sps / max(self.accel_time, 1e-3)) / self.rate
        # Decel: cap target speed by remaining distance so it ramps down.
        decel_cap = (abs(remaining) / self.steps_per_mm) * 1000.0  # ~mm scale
        target = min(max_sps, max(self.fine_sps, decel_cap * 200.0))
        if target > cur:
            cur = min(cur + max_delta, target)
        else:
            cur = max(cur - max_delta, target)
        setattr(self, sps_attr, cur)
        if cur < 1.0:
            cur = self.fine_sps

        n = max(1, int(cur / self.rate))
        n = min(n, abs(remaining))
        positive = remaining > 0
        lgpio.gpio_write(self.h, dir_pin, 1 if positive else 0)
        self._step_burst(step_pin, n, cur)
        setattr(self, pos_attr, pos + (n if positive else -n))
        return True

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
        # Spin in short slices so we can detect when the one-shot move has
        # finished and exit the program automatically.
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node._finished:
            node.get_logger().info("Move complete — shutting down.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
