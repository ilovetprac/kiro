#!/usr/bin/env bash
#
# run.sh — daily startup for the AGV ArUco X/Y alignment.
#
# Automates: free the camera (unload uvcvideo, kill stale nodes), source ROS +
# the workspace, launch the camera + marker node, and start the serial bridge
# to the Arduino. Both run together; Ctrl+C stops everything.
#
# Usage:
#   ./run.sh                 # uses defaults below
#   SERIAL_PORT=/dev/ttyACM0 ./run.sh
#   ARUCO_DICT=DICT_5X5_100 TARGET_ID=3 ./run.sh
#
# Run build.sh first (once, or after pulling new code) to rebuild + flash.

set -e

# ---- Config (override via environment variables) ----
WS="${WS:-$HOME/ros2_ws}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
ARUCO_DICT="${ARUCO_DICT:-DICT_4X4_250}"
TARGET_ID="${TARGET_ID:-1}"

echo "==> Freeing the camera (kill stale nodes, unload uvcvideo)…"
pkill -9 -f realsense2_camera_node 2>/dev/null || true
pkill -9 -f aruco_ir_depth_node   2>/dev/null || true
pkill -9 -f serial_bridge_node    2>/dev/null || true
sudo fuser -k /dev/video* 2>/dev/null || true
sudo modprobe -r uvcvideo 2>/dev/null || true
sleep 1

echo "==> Sourcing ROS 2 + workspace…"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

# Make sure both child processes die when this script exits / Ctrl+C.
cleanup() {
  echo
  echo "==> Shutting down…"
  kill "${CAM_PID}" "${BRIDGE_PID}" 2>/dev/null || true
  pkill -9 -f realsense2_camera_node 2>/dev/null || true
  pkill -9 -f aruco_ir_depth_node   2>/dev/null || true
  pkill -9 -f serial_bridge_node    2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "==> Launching camera + marker node (dict=${ARUCO_DICT}, target_id=${TARGET_ID})…"
ros2 launch qr_coordinates_ros2 aruco_ir_depth.launch.py \
  aruco_dictionary:="${ARUCO_DICT}" target_id:="${TARGET_ID}" 2>/dev/null &
CAM_PID=$!

echo "==> Waiting for the camera to come up…"
sleep 6

echo "==> Starting serial bridge -> Arduino (${SERIAL_PORT})…"
ros2 run qr_coordinates_ros2 serial_bridge_node \
  --ros-args -p serial_port:="${SERIAL_PORT}" &
BRIDGE_PID=$!

echo
echo "============================================================"
echo "Running. Camera+marker PID ${CAM_PID}, bridge PID ${BRIDGE_PID}."
echo "Show ArUco ID ${TARGET_ID} to the camera (near centre, 0.3-1.5 m)."
echo "Press Ctrl+C to stop everything."
echo "============================================================"

# Wait for either child to exit (then cleanup trap fires).
wait
