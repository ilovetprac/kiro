#!/usr/bin/env bash
#
# run.sh — daily startup for the AGV ArUco X/Y alignment (Pi GPIO steppers).
#
# Automates: ensure pigpiod is running, free the camera (unload uvcvideo, kill
# stale nodes), source ROS + the workspace, and launch the full system
# (camera + ArUco marker node + GPIO stepper node) in one go.
# Ctrl+C stops everything.
#
# Usage:
#   ./run.sh
#   INVERT_X=true INVERT_Y=true ./run.sh     # flip an axis that moves backward
#   ARUCO_DICT=DICT_5X5_100 ./run.sh
#
# Run build.sh first (once, or after pulling new code) to rebuild.

set -e

# ---- Config (override via environment variables) ----
WS="${WS:-$HOME/ros2_ws}"
ARUCO_DICT="${ARUCO_DICT:-DICT_4X4_250}"
TARGET_ID="${TARGET_ID:-1}"
INVERT_X="${INVERT_X:-true}"
INVERT_Y="${INVERT_Y:-true}"
STEPS_PER_MM="${STEPS_PER_MM:-1600.0}"
EMITTER="${EMITTER:-1}"          # 1 = IR projector ON (steadier depth/detection)

echo "==> Freeing the camera (kill stale nodes, unload uvcvideo, USB reset)…"
pkill -9 -f realsense2_camera_node 2>/dev/null || true
pkill -9 -f aruco_ir_depth_node   2>/dev/null || true
pkill -9 -f gpio_stepper_node     2>/dev/null || true
sudo fuser -k /dev/video* 2>/dev/null || true

# Fully remove the kernel UVC driver so the RSUSB (libusb) backend can claim
# the D435 cleanly. If uvcvideo (or a stale handle) still holds the device the
# node loops forever on "control_transfer returned error ... Resource
# temporarily unavailable" (errno 11) and never starts streaming.
sudo modprobe -r uvcvideo 2>/dev/null || true
sleep 1

# USB RE-ENUMERATION: the single most reliable cure for the stuck-control-
# transfer (EAGAIN) loop is to power-cycle the D435's USB port so it starts
# from a clean state. We find the Intel RealSense device (VID 8086) and reset
# it via usbreset if available, else via the usb authorized sysfs toggle.
reset_realsense_usb() {
  # Try the `usbreset` helper first (apt: usbutils provides usb-reset on some
  # distros; otherwise this is a no-op and we fall back to sysfs).
  local dev
  dev="$(lsusb 2>/dev/null | grep -i '8086:' | head -n1 | \
         sed -E 's/Bus ([0-9]+) Device ([0-9]+).*/\/dev\/bus\/usb\/\1\/\2/')"
  if [ -n "${dev}" ] && command -v usbreset >/dev/null 2>&1; then
    echo "==> usbreset ${dev}"
    sudo usbreset "${dev}" 2>/dev/null || true
    return
  fi
  # Fallback: toggle the device's 'authorized' flag in sysfs to force a
  # re-enumeration (works without extra tools).
  for d in /sys/bus/usb/devices/*; do
    if [ -f "${d}/idVendor" ] && [ "$(cat "${d}/idVendor" 2>/dev/null)" = "8086" ]; then
      echo "==> USB re-enumerate $(basename "${d}")"
      echo 0 | sudo tee "${d}/authorized" >/dev/null 2>&1 || true
      sleep 1
      echo 1 | sudo tee "${d}/authorized" >/dev/null 2>&1 || true
    fi
  done
}
reset_realsense_usb
# Give the device a moment to re-enumerate before the node opens it.
sleep 3

echo "==> Sourcing ROS 2 + workspace…"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

cleanup() {
  echo
  echo "==> Shutting down…"
  pkill -9 -f realsense2_camera_node 2>/dev/null || true
  pkill -9 -f aruco_ir_depth_node   2>/dev/null || true
  pkill -9 -f gpio_stepper_node     2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Background: once the camera node is up, enable the IR emitter at RUNTIME.
# The launch-arg route is unreliable on some realsense2_camera builds, so we
# set the parameter directly (trying the known name variants).
if [ "${EMITTER}" = "1" ]; then
(
  for i in $(seq 1 30); do
    if ros2 node list 2>/dev/null | grep -q "/camera/camera"; then
      sleep 2
      ros2 param set /camera/camera depth_module.emitter_enabled 1 2>/dev/null \
        && echo "==> IR emitter ON (depth_module.emitter_enabled)" && break
      ros2 param set /camera/camera depth_module.emitter_on_off 1 2>/dev/null \
        && echo "==> IR emitter ON (depth_module.emitter_on_off)" && break
      echo "==> Note: could not set emitter param automatically; "\
           "detection still works, just enable it in realsense-viewer if needed."
      break
    fi
    sleep 1
  done
) &
fi

echo "==> Launching camera + marker + GPIO steppers…"
echo "    dict=${ARUCO_DICT} target_id=${TARGET_ID} invert_x=${INVERT_X} invert_y=${INVERT_Y}"
echo "============================================================"
echo "Show ArUco ID ${TARGET_ID} to the camera (near centre, 0.3-1.5 m)."
echo "Press Ctrl+C to stop everything."
echo "============================================================"

ros2 launch qr_coordinates_ros2 agv_align.launch.py \
  aruco_dictionary:="${ARUCO_DICT}" target_id:="${TARGET_ID}" \
  invert_x:="${INVERT_X}" invert_y:="${INVERT_Y}" \
  steps_per_mm:="${STEPS_PER_MM}" 2>/dev/null
