# qr_coordinates_ros2 — QR Code 3D Coordinates as a ROS 2 Node (D435)

A ROS 2 **Humble** `ament_python` package that detects QR codes from an Intel
RealSense **D435** and **publishes** each QR code's **X, Y, Z position (meters)
relative to the camera** plus the **decoded QR ID**.

Built and tuned for a **Raspberry Pi 4B (4GB) / Ubuntu 22.04 / ROS 2 Humble**.

## Why this instead of the standalone script?

The standalone `camera display/qr_coordinates.py` uses `pyrealsense2`, which has
**no pip wheel on ARM64** (you'd have to build librealsense from source on the
Pi). This ROS 2 node instead subscribes to the official **`realsense2_camera`**
driver's topics and does the 3D deprojection from `camera_info` intrinsics — so
it needs **no `pyrealsense2`** and fits naturally into a ROS 2 robot.

## What it publishes

| Topic                               | Type                          | Description                         |
|-------------------------------------|-------------------------------|-------------------------------------|
| `~/qr/point`                        | `geometry_msgs/PointStamped`  | QR center in 3D (m), color frame    |
| `~/qr/id`                           | `std_msgs/String`             | Decoded QR data / ID                |

It also logs each detection to the ROS console:
```
QR ID: 'box-42' | X: +0.051 m  Y: -0.012 m  Z: +0.734 m
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

- **No detections / no output** — Confirm topics exist: `ros2 topic list | grep camera`.
  Make sure `align_depth.enable:=true` is set (the provided launch does this).
- **"no valid depth at pixel ..."** — QR too close/far/reflective. Keep it 0.3–3 m.
- **Sync drops everything** — Increase `sync_slop` (e.g. `0.1`) if color/depth
  stamps drift on a loaded Pi.
- **Choppy on the Pi** — Lower resolution/fps in the launch file, or run headless.
