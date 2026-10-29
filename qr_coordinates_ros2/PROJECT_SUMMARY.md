# AGV ArUco X/Y Alignment — Project Summary & Build Log

A complete record of this project: what it does, the hardware, the full software
design, every setting, the wiring, the troubleshooting journey, and how to run
it. Exported as a single reference document.

---

## 1. Goal

Align the **female connector on an AGV** to a fixed **male connector** at a
parking station. A camera on the AGV sees an **ArUco marker** near the male
connector and reports the marker's X/Y offset; two stepper-driven screw slides
move the female connector so the marker is centred (X→0, Y→0). The AGV then
reverses to mate (Z handled by the AGV, ignored here).

- **+X = right, +Y = up** (Cartesian), origin = camera optical axis.
- Marker centred in view ⇒ X ≈ 0, Y ≈ 0.
- Only **X and Y** are used for control; Z (depth) is used only to convert
  pixels → millimetres.

---

## 2. Hardware

| Item | Detail |
|------|--------|
| Compute | Raspberry Pi 4B (4GB), Ubuntu 22.04, ROS 2 Humble (ARM64) |
| Camera | Intel RealSense **D435** (uses **IR/infra1 + depth**, RGB off) |
| Motors | 2× NEMA 11, 1.8°/step (200 steps/rev), ~0.6–0.8 A, 24 V |
| Screw | T6 lead screw, **1 mm lead**, 150 mm stroke |
| Drivers | 2× TB6600-style **microstep drivers** (STEP/DIR), 9–42 VDC |
| Power | 24 V DC supply to drivers (NOT to the Pi) |
| Control | Pi GPIO → drivers directly (no Arduino) via **lgpio** |

**Motor wiring (YUN DUAN label):** A+ = Black, A− = Green, B+ = Red, B− = Blue.

**Driver DIP switches (both drivers identical):**
- Microstep 1/8 = 1600 pulse/rev: **SW1 OFF, SW2 ON, SW3 OFF**
- Current 0.5 A (PK 0.7 A): **SW4 ON, SW5 ON, SW6 ON**
- 1 mm lead × 1600 pulse/rev = **1600 steps/mm** (0.000625 mm/step).

**Pi GPIO → driver signals (common-cathode: drive `+`, tie `−` to Pi GND):**
| Signal | Pi BCM | Physical pin | Driver |
|--------|--------|--------------|--------|
| X STEP | GPIO17 | 11 | X PUL+ |
| X DIR  | GPIO27 | 13 | X DIR+ |
| Y STEP | GPIO22 | 15 | Y PUL+ |
| Y DIR  | GPIO23 | 16 | Y DIR+ |
| GND    | —      | 9/14 | all PUL−/DIR− |

**Power/motor (never to the Pi):** 24 V → VCC/GND (both drivers); coils → A/B.

> 3.3 V vs 5 V: driver inputs are labelled +5V but the Pi is 3.3 V. TB6600 optos
> usually trigger at 3.3 V; add a 3.3→5 V level shifter on PUL+/DIR+ if steps
> are missed.

---

## 3. Software architecture (ROS 2 package `qr_coordinates_ros2`)

```
D435 (realsense2_camera)
   infra1 IR image + depth + camera_info
        |
   aruco_ir_depth_node     detect ArUco (ID 1) in IR, read depth at marker
        |                  centre, deproject -> X/Y/Z mm (Cartesian, +Y up)
        |  publishes:
        |    ~/marker/id      std_msgs/Int32
        |    ~/marker/point   geometry_msgs/PointStamped (m)
        |    ~/marker/xy_mm   geometry_msgs/Point (mm)       <- used for control
        |    ~/marker/info    std_msgs/String (human-readable)
        |    ~/marker/present std_msgs/Bool (True/False every frame)
        v
   gpio_stepper_node        chase latest X/Y -> STEP/DIR pulses on Pi GPIO (lgpio)
        v
   2 microstep drivers -> 2 steppers -> 2 screw slides
```

Launch everything: **`agv_align.launch.py`** (camera + marker node + stepper).

### Key files
- `qr_coordinates_ros2/qr_coordinates_ros2/aruco_ir_depth_node.py` — vision
- `qr_coordinates_ros2/qr_coordinates_ros2/gpio_stepper_node.py` — motor control
- `qr_coordinates_ros2/launch/agv_align.launch.py` — full launch
- `qr_coordinates_ros2/launch/aruco_ir_depth.launch.py` — camera + marker only
- `qr_coordinates_ros2/scripts/run.sh` — one-command startup
- `qr_coordinates_ros2/scripts/build.sh` — pull + rebuild
- `qr_coordinates_ros2/WIRING_GPIO.md` — wiring reference
- `qr_coordinates_ros2/TROUBLESHOOTING_PI.md` — D435-on-Pi fixes

---

## 4. Behaviour / control logic

Motors move **only when ALL** are true (vision node gates, per frame):
1. Marker **ID 1** detected (others ignored; `target_id` = −1 = any)
2. Seen for **2 consecutive frames** (`confirm_frames`)
3. **Valid depth** at the marker centre
4. Both **|X| ≤ 70 mm and |Y| ≤ 70 mm** (`max_offset_mm`; else rejected, no move)

Then the stepper moves when:
5. **`~/marker/present` = True** (stops IMMEDIATELY when False — no timeout)
6. Offset **> deadband** (0.5 mm)
7. Slide within **±70 mm** of power-on centre (`max_travel_mm`)

**Two-speed:** >5 mm from target = fast; ≤5 mm = fine; ≤0.5 mm = hold.
**No lock/latch** — continuous real-time correction; always chases the latest
reading; stops instantly when the marker disappears.

---

## 5. Current parameter values

### Vision node (`aruco_ir_depth_node`)
| Param | Value | Meaning |
|-------|-------|---------|
| aruco_dictionary | DICT_4X4_250 | marker dictionary (4×4 inner grid) |
| target_id | 1 | only act on marker ID 1 (−1 = any) |
| max_offset_mm | 70.0 | reject if |X| or |Y| beyond this |
| confirm_frames | 2 | consecutive detections before acting |
| smoothing | 0.1 | EMA on X/Y/Z (0 = none) |
| depth_window | 20 | depth median patch half-size (px) |
| print_rate_hz | 30.0 | console/topic print rate |

### Stepper node (`gpio_stepper_node`)
| Param | Value | Meaning |
|-------|-------|---------|
| steps_per_mm | 1600.0 | 1/8 microstep × 1 mm lead |
| deadband_mm | 0.5 | within this = aligned/hold |
| fine_zone_mm | 5.0 | within this = fine speed |
| fast_sps | 8000 | step rate when far (was 4000; 12000 was too rough) |
| fine_sps | 1600 | step rate when near (was 800) |
| max_travel_mm | 70.0 | slide clamp ± from power-on centre |
| invert_x / invert_y | false / false | flip axis direction |
| control_rate_hz | 100.0 | control loop rate |
| x_step/x_dir/y_step/y_dir | 17/27/22/23 | BCM GPIO pins |

---

## 6. How to run

One-time:
```bash
sudo apt install -y python3-lgpio          # GPIO (kernel gpiochip, no daemon)
cd ~/kiro && git pull origin main
chmod +x ~/kiro/qr_coordinates_ros2/scripts/*.sh
```

Build (after pulling code):
```bash
cd ~/kiro/qr_coordinates_ros2/scripts && ./build.sh
```

Run (camera + marker + steppers, one command):
```bash
./run.sh
# options:
INVERT_X=true INVERT_Y=true ./run.sh      # flip an axis
EMITTER=0 ./run.sh                         # IR projector off
ARUCO_DICT=DICT_5X5_100 ./run.sh
```

Before first motion: **centre both slides** (no end-stops → power-on = 0,0),
24 V on, slides free, hand on the switch. If an axis drives the wrong way, set
its invert flag.

---

## 7. Troubleshooting journey (what we solved)

1. **`UVCIOC_CTRL_QUERY ... timed out` on the Pi** → old kernel UVC backend.
   Fixed by rebuilding **librealsense with the RSUSB backend**
   (`FORCE_RSUSB_BACKEND=true`).
2. **Library version mismatch / "null pointer block"** → rebuilt the
   realsense-ros wrapper against the new librealsense (2.59.1), removed the apt
   2.58 packages.
3. **Camera brown-out / disconnect with RGB+depth** → switched to **IR+depth**
   (lower power); also needs the device left in a good USB state.
4. **NumPy 2.x vs cv_bridge** → pinned `numpy<2`.
5. **QR vs ArUco** → user's tags are **ArUco 4×4**, switched detector to
   `cv2.aruco`.
6. **No X/Y output** → QoS mismatch (RealSense images are BEST_EFFORT) and the
   aligned-depth topic wasn't produced → used plain depth + latest-depth pairing.
7. **"no valid depth"** → median depth over a window + enabled the **IR emitter**.
8. **Dropped the Arduino** → direct **Pi GPIO** via **lgpio** (pigpio has no
   Ubuntu 22.04 package).
9. **Motor behaviour tuning** → non-blocking chase, small step increments,
   two-speed, stop-immediately-on-loss via `~/marker/present`, speed 3×/2×
   then eased to 8000/1600 for smoothness.

---

## 8. Known limitations / next steps

- **No end-stops / homing.** Position is tracked relative to power-on; centre
  the slides before each run. Add limit switches for auto-homing later.
- **Corners:** depth can drop out and X/Y can exceed ±70 mm near the image
  corners, so the marker may not be acted on there. Options discussed: fixed-Z
  deprojection (if camera-to-marker distance is ~constant) and/or widening the
  range — not yet implemented.
- **lgpio software-timed pulses** can sound rough at high step rates; 8000 sps
  is the current smooth setting. Acceleration ramping would allow higher smooth
  speeds (not implemented).
- **Connector vs marker offset:** alignment targets the MARKER centre. If the
  marker is not exactly on the male-connector axis, add a target offset so the
  CONNECTORS align (not yet implemented — needs the marker→connector offset).

---

## 9. Repository

GitHub: **ilovetprac/kiro**, package at `qr_coordinates_ros2/`.
This whole system was built and iterated via pull requests #1–#51.
