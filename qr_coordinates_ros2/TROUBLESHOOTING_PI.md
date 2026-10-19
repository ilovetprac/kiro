# Troubleshooting: D435 on Raspberry Pi 4B / Ubuntu 22.04 / ROS 2 Humble

This documents the `UVCIOC_CTRL_QUERY ... Connection timed out` failure seen when
bringing up the D435 with `realsense2_camera` on a Raspberry Pi 4B, and the fix.

## Symptom

The camera is detected and a stream profile opens, but then the log fills with:

```
get_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed on control 1 Last Error: Connection timed out
get_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed on control 7 Last Error: No such file or directory
No intrinsics available for this stream profile. Using zeroed intrinsics as default.
Error starting device: std::exception
```

## How we diagnosed it

1. **`rs-enumerate-devices -c` works** — static queries succeed, intrinsics/extrinsics
   are all readable. So the camera, cable, and power deliver enough to enumerate.
2. **Depth-only at 640x480 fails** the same way on control 1/7.
3. **Color-only at 640x480 fails** the same way on control 1.

Because BOTH minimal single-stream configs fail identically on the UVC
**extension-unit (XU) control** transfers, the cause is **not** USB bandwidth or
power (a powered hub does not help). It is the **kernel UVC driver** mishandling
XU control ioctls.

4. **Kernel: `5.15.0-1110-raspi`** — newer than the versions librealsense's V4L2
   backend is patched for. Unpatched kernel UVC = `UVCIOC_CTRL_QUERY` timeouts.
5. **Firmware: `5.11.1.100`** — very old (current D435 production FW is 5.13.x+),
   which compounds XU-protocol mismatches with recent librealsense.

Reference: [realsense-ros #3089 — UVCIOC_CTRL_QUERY Error on Raspberry Pi 4](https://github.com/IntelRealSense/realsense-ros/issues/3089).

## Fix 1 (primary): rebuild librealsense with the RSUSB backend

The RSUSB backend uses libusb to talk to the camera directly, **bypassing the
kernel UVC driver** and the broken `UVCIOC_CTRL_QUERY` path. Run the helper:

```bash
cd ~/ros2_ws/src/qr_coordinates_ros2/scripts   # or wherever you cloned it
chmod +x build_librealsense_rsusb.sh
./build_librealsense_rsusb.sh
```

Then test **without ROS** first:

```bash
# unplug + replug the camera, then:
rs-enumerate-devices
realsense-viewer      # enable Depth + Color — expect NO UVCIOC_CTRL_QUERY timeouts
```

If the viewer streams cleanly, rebuild `realsense-ros` against this librealsense
(from its own source tree) and re-run the ROS launch.

## Fix 2 (also recommended): update the camera firmware

FW `5.11.1.100` is ancient. Update to current D435 production firmware (5.13.x+),
easiest from a Windows/Linux PC with the RealSense Viewer or `rs-fw-update`:

```bash
# Replace the path with a real downloaded .bin — do NOT type the angle brackets.
rs-fw-update -f Signed_Image_UVC_5_13_0_50.bin
```

Firmware downloads: Intel D400 firmware page
(<https://www.intel.com/content/www/us/en/download/19242/firmware-for-intel-realsense-d400-product-family.html>).

## Useful checks

```bash
uname -r                       # kernel version
sudo dmesg | grep -i uvc | tail   # kernel UVC complaints (needs sudo)
pkill -9 -f realsense2_camera_node   # clear a stale camera node holding /dev/video*
```

## `control_transfer ... Resource temporarily unavailable (errno 11)` — harmless

After switching to the RSUSB backend you may see a continuous stream of:

```
WARNING (messenger-libusb.cpp:42) control_transfer returned error, index: 768,
  error: Resource temporarily unavailable, number: 11
```

These are **WARNINGS, not errors** — background control-channel retries on the
RSUSB/libusb backend. **They do NOT stop image streaming.** Confirm frames are
actually flowing:

```bash
ros2 topic hz /camera/camera/color/image_raw   # ~26 Hz = streaming fine
```

If `topic hz` shows a rate, ignore the warnings — the camera is working.

## `enable_sync` / `initial_reset` make the device fail to start (Pi 4B)

On the Pi 4B with the RSUSB backend and FW 5.17.x, launching with
`enable_sync:=true` **or** `initial_reset:=true` caused the device to fail to
start (null-pointer/"block" error, or a hang). With **both OFF** the camera
streams normally. The launch file in this repo now defaults to
`enable_sync:=false` and `initial_reset:=false` for this reason.

## Note

The ROS 2 package and launch file in this repo are not the cause of this error —
the node builds, connects to the correct topics, and waits for frames. The
blocker is below ROS, between librealsense, the camera firmware, and the kernel
UVC driver. No launch-file change can fix a UVC control-transfer timeout.
