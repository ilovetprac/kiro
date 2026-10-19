# qr_coordinates_ros2 — ArUco Marker 3D Coordinates as a ROS 2 Node (D435)

A ROS 2 **Humble** `ament_python` package that detects **ArUco markers** from an
Intel RealSense **D435** and **publishes** each marker's **X, Y, Z position
(meters) relative to the camera** plus its integer **marker ID**.

> Note: the package/node keep the historical name `qr_coordinates`, but they
> detect **ArUco markers** (via `cv2.aruco`), not QR codes. ArUco markers carry
> a numeric ID and are much more robust to detect.

Built and tuned for a **Raspberry Pi 4B (4GB) / Ubuntu 22.04 / ROS 2 Humble**.

## ArUco dictionary

Set the `aruco_dictionary` param/arg to match your markers (default
`DICT_4X4_50`). Common values: `DICT_4X4_50`, `DICT_4X4_100`, `DICT_5X5_100`,
`DICT_6X6_250`, `DICT_7X7_250`, `DICT_APRILTAG_36h11`.

```bash
# e.g. if your markers are from the 5x5 dictionary:
ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py aruco_dictionary:=DICT_5X5_100
```

> Requires the `aruco` module from OpenCV contrib. Ubuntu's `python3-opencv`
> (4.5.4) includes it. Verify with:
> `python3 -c "import cv2; print(hasattr(cv2, 'aruco'))"`  → should print `True`.

## Why this instead of the standalone script?

The standalone `camera display/qr_coordinates.py` uses `pyrealsense2`, which has
**no pip wheel on ARM64** (you'd have to build librealsense from source on the
Pi). This ROS 2 node instead subscribes to the official **`realsense2_camera`**
driver's topics and does the 3D deprojection from `camera_info` intrinsics — so
it needs **no `pyrealsense2`** and fits naturally into a ROS 2 robot.

## 2D vs 3D mode

The node has a `mode` parameter so you can run it with or without depth:

| Mode    | Needs depth? | Publishes                                   | Use when |
|---------|--------------|---------------------------------------------|----------|
| `2d`    | No (color only) | marker **ID** + **pixel** (u, v)         | Depth can't start (e.g. Pi 4B without a powered USB hub) |
| `3d`    | Yes          | marker **ID** + **3D point** (X, Y, Z, m)   | Depth is streaming reliably |
| `auto`  | Optional     | 2D until depth appears, then 3D             | General use |

**The launch file defaults to `2d` with the depth stream off**, so it runs on a
Pi 4B without a powered hub. To get 3D once you have adequate power:

```bash
# 2D (default) — color only, no depth stream started:
ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py

# 3D — starts depth and reports X/Y/Z (needs a powered USB hub on a Pi 4B):
ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py mode:=3d enable_depth:=true
```

## What it publishes

| Topic                               | Type                          | Description                              |
|-------------------------------------|-------------------------------|------------------------------------------|
| `~/marker/id`                       | `std_msgs/Int32`              | Detected ArUco marker ID (always)        |
| `~/marker/pixel`                    | `geometry_msgs/PointStamped`  | Marker center pixel (x=u, y=v, z=0)      |
| `~/marker/point`                    | `geometry_msgs/PointStamped`  | Marker center in 3D (m), color frame (3D)|

Fully-qualified topic names are `/qr_coordinates_node/marker/{id,pixel,point}`.

It also logs each detection to the ROS console:
```
Marker ID: 7 | X: +0.051 m  Y: -0.012 m  Z: +0.734 m
```

Coordinate frame (optical frame, meters): +X right, +Y down, +Z forward.

## Prerequisites (Raspberry Pi 4B / Ubuntu 22.04 / Humble)

Install the RealSense ROS 2 driver and build helpers from apt (ARM64 wheels
exist for OpenCV/NumPy, so no source build of librealsense is required here):

```bash
sudo apt update
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  python3-opencv python3-numpy \
  python3-colcon-common-extensions
```

> Plug the D435 into a **blue USB 3.0** port and use a solid **5V/3A** supply.
> On a 4GB Pi, keep streams modest (the launch file defaults to 640x480x30).

## Build

Put this package in a colcon workspace, then build:

```bash
mkdir -p ~/ros2_ws/src
cp -r qr_coordinates_ros2 ~/ros2_ws/src/     # or git clone into src/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qr_coordinates_ros2
source install/setup.bash
```

## Run

Easiest — one launch starts the camera driver (with depth aligned to color)
and the node together:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py
```

If you already run `realsense2_camera` elsewhere, skip starting a second one:

```bash
ros2 launch qr_coordinates_ros2 qr_coordinates.launch.py start_camera:=false
```

Or run just the node (camera must be publishing already):

```bash
ros2 run qr_coordinates_ros2 qr_coordinates_node
```

## Inspect the output

```bash
ros2 topic echo /qr_coordinates_node/qr/point
ros2 topic echo /qr_coordinates_node/qr/id
```

## Parameters

| Parameter            | Default                                                  | Description                              |
|----------------------|----------------------------------------------------------|------------------------------------------|
| `color_topic`        | `/camera/camera/color/image_raw`                         | Color image topic                        |
| `depth_topic`        | `/camera/camera/aligned_depth_to_color/image_raw`        | Aligned depth topic (16UC1, mm)          |
| `camera_info_topic`  | `/camera/camera/color/camera_info`                       | Color intrinsics                         |
| `sync_slop`          | `0.05`                                                   | Max sync time diff (s) across topics     |

> The Humble `realsense2_camera` default namespace is `/camera/camera`. If your
> launch uses a different one, override these parameters, e.g.:
> ```bash
> ros2 run qr_coordinates_ros2 qr_coordinates_node \
>   --ros-args -p color_topic:=/my_cam/color/image_raw
> ```

## Troubleshooting

- **`AttributeError: _ARRAY_API not found` / "module compiled using NumPy 1.x
  cannot be run in NumPy 2.x"** — Ubuntu's `cv_bridge`/`python3-opencv` are built
  against NumPy 1.x. If a NumPy 2.x got installed via pip, pin it back:
  ```bash
  pip3 install "numpy<2"
  python3 -c "import numpy; print(numpy.__version__)"   # must be 1.x
  ```
- **`xioctl(UVCIOC_CTRL_QUERY) failed ... Connection timed out` / "No intrinsics
  available ... Using zeroed intrinsics"** — The driver can't talk to the camera.
  First confirm the camera itself works outside ROS with
  `rs-enumerate-devices -c` (install `librealsense2-utils`). If that succeeds,
  the issue is usually a flaky USB port/cable or power — try another blue USB 3.0
  port, a known-good USB 3 data cable, and the official 5V/3A supply.
- **No detections / no output** — Confirm topics exist: `ros2 topic list | grep camera`.
  Make sure `align_depth.enable:=true` is set (the provided launch does this).
- **"no valid depth at pixel ..."** — QR too close/far/reflective. Keep it 0.3–3 m.
- **Sync drops everything** — Increase `sync_slop` (e.g. `0.1`) if color/depth
  stamps drift on a loaded Pi.
- **Choppy on the Pi** — Lower resolution/fps in the launch file, or run headless.
