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

# Auto-detect the ROS 2 distro (works on Jazzy/24.04 and Humble/22.04).
# Override with ROS_DISTRO=<name> if needed.
detect_ros_distro() {
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "${ROS_DISTRO}"; return
  fi
  for d in jazzy humble iron rolling; do
    if [ -f "/opt/ros/${d}/setup.bash" ]; then echo "${d}"; return; fi
  done
  echo ""
}
ROS="$(detect_ros_distro)"
if [ -z "${ROS}" ]; then
  echo "ERROR: no ROS 2 distro found in /opt/ros/. Install ROS 2 first." >&2
  exit 1
fi

ARUCO_DICT="${ARUCO_DICT:-DICT_4X4_250}"
TARGET_ID="${TARGET_ID:-1}"
INVERT_X="${INVERT_X:-true}"
INVERT_Y="${INVERT_Y:-false}"
STEPS_PER_MM="${STEPS_PER_MM:-1600.0}"
EMITTER="${EMITTER:-1}"          # 1 = IR projector ON (steadier depth/detection)

echo "==> Freeing the camera (kill stale nodes, unload uvcvideo, USB reset)…"
# No sudo password prompts: run ./install_nopasswd.sh ONCE to allow the few
# camera-freeing commands below to run password-free. If that rule is not yet
# installed, warn once (sudo will then prompt as usual for this run).
if ! sudo -n true 2>/dev/null; then
  echo "    NOTE: sudo will prompt for a password. To avoid this every time,"
  echo "          run:  ./install_nopasswd.sh   (one-time setup)."
fi

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

echo "==> Sourcing ROS 2 (${ROS}) + workspace…"
source "/opt/ros/${ROS}/setup.bash"
source "${WS}/install/setup.bash"

LAUNCH_PID=""
cleanup() {
  echo
  echo "==> Shutting down…"
  if [ -n "${LAUNCH_PID}" ]; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  fi
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
echo "Show ArUco ID ${TARGET_ID} to the camera (within 25 cm)."
echo "After 'correction complete', press Enter to return X/Y to origin"
echo "(0,0) and end the program. Ctrl+C = hard stop (slides stay put)."
echo "============================================================"

# Run the launch in the BACKGROUND (stdin from /dev/null) so this script can
# read the Enter key itself. ros2 launch does not pass the terminal's stdin to
# its nodes, so the node cannot read Enter directly.
ros2 launch qr_coordinates_ros2 agv_align.launch.py \
  aruco_dictionary:="${ARUCO_DICT}" target_id:="${TARGET_ID}" \
  invert_x:="${INVERT_X}" invert_y:="${INVERT_Y}" \
  steps_per_mm:="${STEPS_PER_MM}" < /dev/null 2>/dev/null &
LAUNCH_PID=$!

# Each Enter press sends SIGUSR1 to the stepper node = "return to origin".
# The node ignores it until the correction is complete. When the slides are
# back at (0,0) the node exits, the launch shuts down, and this loop ends.
if [ -t 0 ]; then
  while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
    if read -r -t 1 _; then
      # Signal ONLY the stepper node started by this launch (a broad
      # `pkill -f` could hit any process whose command line mentions the
      # name, and SIGUSR1's default action is to terminate it).
      pkill -USR1 -P "${LAUNCH_PID}" -f gpio_stepper_node 2>/dev/null \
        || pkill -USR1 -f "lib/qr_coordinates_ros2/gpio_stepper_node" 2>/dev/null \
        || true
    fi
  done
fi
wait "${LAUNCH_PID}" 2>/dev/null || true
LAUNCH_PID=""
