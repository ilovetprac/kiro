# d435_marker_xy

ROS 2 **Humble** node that continuously publishes an **ArUco marker's X, Y
position (in millimeters)** relative to an **Intel RealSense D435**, using the
**depth + infrared streams only — no RGB**.

The marker center is the **origin of a cartesian plane**: when the camera is
pointed straight at the marker (marker on the optical axis), the readout is
**(X = 0, Y = 0)**.

- **X**: right `+`, left `−`
- **Y**: up `+`, down `−`
- **Z** (forward distance) is also published, for reference
- **Units**: millimeters (mm)

---

## How it works

- **Detection** uses the D435's **left infrared (IR)** stream. The IR imager
  sees the printed black/white ArUco pattern; the **color camera is never
  enabled**.
- **Distance** uses the **depth** stream, aligned to the IR frame.
- The marker's center pixel + its depth are **deprojected** into a real-world
  3D point using the camera intrinsics. Because that point is measured from the
  optical axis, "camera centered on the marker" ⇒ `(0, 0)` with **no manual
  origin calibration**.

### Your marker

A `DICT_4X4_50` ArUco marker whose **full printed square (including the white
border) is 10 cm**. The "4×4" is the inner code grid — the surrounding border
is normal for *every* ArUco marker, so this is still `DICT_4X4_50` (not 6×6).
The X/Y readout comes straight from depth and does **not** need the marker's
physical size.

### D435 emitter note

The D435's IR **projector** sprays a dot pattern that speckles the IR image and
corrupts ArUco detection, so the emitter is **OFF by default**
(`emitter_enabled: false`). Depth still works from ambient light indoors. If you
need the projector for better depth range, set `emitter_enabled: true` — but
detection may degrade.

---

## Install (on the Raspberry Pi 4 / Ubuntu 22.04 / ROS 2 Humble)

```bash
# 1) Python deps (not available via rosdep):
pip3 install pyrealsense2 opencv-contrib-python numpy
#    NOTE: use opencv-contrib-python (it includes cv2.aruco); plain
#    opencv-python does NOT.

# 2) Put this package in a colcon workspace and build:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ilovetprac/d435_marker_xy.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # optional
colcon build --packages-select d435_marker_xy
source install/setup.bash
```

> The code auto-detects the OpenCV ArUco API, so it works on both OpenCV ≥ 4.7
> (pip's `opencv-contrib-python`) and ≤ 4.6 (Humble's system OpenCV 4.5.x).

---

## Run

```bash
# with defaults (tracks the first marker it sees):
ros2 run d435_marker_xy marker_xy_node

# lock onto a specific marker ID:
ros2 run d435_marker_xy marker_xy_node --ros-args -p marker_id:=0

# via launch file (loads config/params.yaml):
ros2 launch d435_marker_xy marker_xy.launch.py
```

Live log output (updates continuously):

```
[d435_marker_xy]: Marker 0:  X=   +12.3 mm  Y=   -45.6 mm  (Z=  612.0 mm)
```

---

## Published topics

| Topic                         | Type                              | Description                                                        |
|-------------------------------|-----------------------------------|--------------------------------------------------------------------|
| `~/marker_point`              | `geometry_msgs/PointStamped`      | X, Y, Z in **meters** (ROS SI units; +X right, +Y up, +Z forward). |
| `~/marker_xy_mm`              | `std_msgs/Float32MultiArray`      | `[X_mm, Y_mm, Z_mm]`.                                              |
| `~/marker_found`              | `std_msgs/Bool`                   | `True` when the target marker is detected with valid depth.        |

Echo them with, e.g.:

```bash
ros2 topic echo /d435_marker_xy/marker_xy_mm
ros2 topic echo /d435_marker_xy/marker_point
```

---

## Parameters

| Parameter          | Type    | Default                     | Description                                              |
|--------------------|---------|-----------------------------|----------------------------------------------------------|
| `aruco_dict`       | string  | `DICT_4X4_50`               | ArUco dictionary name.                                   |
| `marker_id`        | int     | `-1`                        | `>=0` tracks only that ID; `-1` = first marker seen.     |
| `width`            | int     | `640`                       | Stream width.                                            |
| `height`           | int     | `480`                       | Stream height.                                           |
| `fps`              | int     | `30`                        | Stream frame rate.                                       |
| `emitter_enabled`  | bool    | `false`                     | IR projector on/off (off = clean ArUco detection).       |
| `depth_window`     | int     | `2`                         | Half-window (px) for median depth filtering at center.   |
| `publish_rate_hz`  | double  | `30.0`                      | Publish / processing rate.                               |
| `frame_id`         | string  | `d435_ir_optical_frame`     | Frame id stamped on `PointStamped`.                      |
| `marker_size_mm`   | double  | `100.0`                     | Full printed square incl. border (10 cm).                |
| `log_to_console`   | bool    | `true`                      | Print the live `X=.. Y=..` line to the ROS log.          |

---

## License

MIT
