# Migrating the Pi to Ubuntu 24.04 + ROS 2 Jazzy

Goal: move the Raspberry Pi 4B from **Ubuntu 22.04 + ROS 2 Humble** to
**Ubuntu 24.04 + ROS 2 Jazzy** so it matches the mini PC (native pub/sub, no
cross-distro DDS tricks).

> ⚠️ **This rebuilds the whole camera stack** (source-built librealsense +
> realsense-ros). Budget time for it and KEEP YOUR WORKING 22.04 CARD as a
> fallback. Do NOT do an in-place `do-release-upgrade` on the working card —
> flash a FRESH card instead.

---

## 0. Back up (do this first, do not skip)

- **Keep the current 22.04 SD card untouched.** Flash Jazzy onto a DIFFERENT
  card so you can swap back instantly if anything fails.
- Note down what's irreplaceable on the Pi (any local-only config, calibration,
  tweaks). The code lives in GitHub (`ilovetprac/kiro`), so that's safe.
- Record the current working versions for reference:
  ```bash
  # on the CURRENT 22.04 Pi, before wiping:
  dpkg -l | grep -i realsense
  rs-enumerate-devices | grep -i firmware     # D435 firmware version
  uname -a                                    # kernel
  ```

---

## 1. Flash Ubuntu 24.04 (64-bit) for Raspberry Pi

Use Raspberry Pi Imager (or the Ubuntu 24.04.x **Server or Desktop ARM64**
image for Pi). After first boot:
```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl build-essential cmake pkg-config
```

---

## 2. Install ROS 2 Jazzy

```bash
# locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS 2 apt repo
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Jazzy (ros-base is enough on a headless Pi; add rqt/rviz only if you want GUI)
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-compressed-image-transport
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo $ROS_DISTRO     # jazzy
```

---

## 3. GPIO: lgpio on 24.04

```bash
sudo apt install -y python3-lgpio
# confirm it imports:
python3 -c "import lgpio; print('lgpio OK')"
```
The GPIO group/permissions are the same as before (you're in `dialout`). Test
a pin pulse exactly as on 22.04 if a motor seems dead.

---

## 4. Rebuild librealsense (RSUSB backend) on 24.04

This is the big step — same procedure as 22.04, just on the new OS. Use the
repo script (it is OS-version-agnostic):
```bash
cd ~ && git clone https://github.com/ilovetprac/kiro.git   # if not present
cd ~/kiro/qr_coordinates_ros2/scripts
chmod +x build_librealsense_rsusb.sh
./build_librealsense_rsusb.sh
```
Notes for 24.04:
- The script installs deps, adds temp swap (4GB Pi), builds with
  `FORCE_RSUSB_BACKEND=true`, installs udev rules, removes swap.
- If a newer librealsense is needed for 24.04 toolchain compatibility, check out
  a recent tag inside `~/librealsense` before building (the script has a
  commented `git checkout` line). Build the latest release tag if the default
  branch fails to compile on 24.04's GCC.
- After it finishes: unplug/replug the D435 and test WITHOUT ROS:
  ```bash
  rs-enumerate-devices
  realsense-viewer          # enable Depth + IR; confirm no XU timeouts
  ```
  If the D435 firmware differs, update it with `rs-fw-update` as before.

---

## 5. Build realsense-ros for Jazzy

Install the Jazzy realsense2_camera from apt if available (simplest):
```bash
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
```
If the apt package links against a DIFFERENT librealsense than your RSUSB
build, build realsense-ros **from source against your source-built
librealsense** instead:
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro jazzy --skip-keys=librealsense2 -y
colcon build --cmake-args -DBUILD_WITH_RSUSB_BACKEND=ON
source install/setup.bash
```

---

## 6. Build this package (qr_coordinates_ros2) on Jazzy

```bash
cd ~/kiro/qr_coordinates_ros2/scripts
./build.sh          # copies to ~/ros2_ws/src, colcon build, source
```
`build.sh` sources whatever ROS is active; on the Jazzy card it builds against
Jazzy. The Python nodes (rclpy / cv_bridge / std_msgs / sensor_msgs /
geometry_msgs) are distro-agnostic and should build unchanged.

---

## 7. Verify the full pipeline

```bash
cd ~/kiro/qr_coordinates_ros2/scripts
./run.sh
# in a 2nd terminal:
ros2 topic hz /camera/camera/infra1/image_rect_raw   # ~30 Hz = camera OK
ros2 topic echo /aruco_ir_depth_node/marker/status
```

---

## 8. Networking to the mini PC (now trivial — both Jazzy)

Both machines are Jazzy now, so pub/sub is native. On BOTH:
```bash
export ROS_DOMAIN_ID=7     # same value on both
```
Then stream + subscribe exactly as in NETWORK_STREAM.md (the compressed JPEG
stream still applies for WiFi). No CycloneDDS forcing needed — same distro.

---

## Likely trouble spots on 24.04 (and fixes)

| Symptom | Fix |
|---------|-----|
| librealsense fails to compile (GCC 13 on 24.04) | check out the latest librealsense release tag before building |
| `realsense2_camera` apt pkg uses V4L2 backend (XU timeouts return) | build realsense-ros from source with `BUILD_WITH_RSUSB_BACKEND=ON` against your RSUSB librealsense |
| control_transfer errno 11 loop / no topic | same as before — run.sh frees + USB-re-enumerates the device; keep `initial_reset:=false` |
| NumPy 2 breaks cv_bridge/opencv | pin `numpy<2` in the venv/env if you hit ABI errors (as on 22.04) |
| `python3-lgpio` missing | `sudo apt install python3-lgpio` |

## Rollback
If anything blocks you, power off and swap back to the **22.04 Humble SD card** —
it is untouched and still works. Then tell me what failed and we debug the
Jazzy card without downtime.
