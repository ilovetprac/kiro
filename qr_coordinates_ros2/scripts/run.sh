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
INVERT_X="${INVERT_X:-false}"
INVERT_Y="${INVERT_Y:-false}"
STEPS_PER_MM="${STEPS_PER_MM:-1600.0}"
EMITTER="${EMITTER:-1}"          # 1 = IR projector ON (steadier depth/detection)

echo "==> Freeing the camera (kill stale nodes, unload uvcvideo)…"
pkill -9 -f realsense2_camera_node 2>/dev/null || true
pkill -9 -f aruco_ir_depth_node   2>/dev/null || true
pkill -9 -f gpio_stepper_node     2>/dev/null || true
sudo fuser -k /dev/video* 2>/dev/null || true
sudo modprobe -r uvcvideo 2>/dev/null || true
sleep 1

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
