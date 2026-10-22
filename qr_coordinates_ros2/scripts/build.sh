#!/usr/bin/env bash
#
# build.sh — pull latest code, rebuild the ROS 2 package, and (optionally)
# flash the Arduino. Run this once after pulling new changes; then use run.sh
# for day-to-day startup.
#
# Usage:
#   ./build.sh            # pull + copy + colcon build + flash Arduino
#   ./build.sh --no-flash # skip the Arduino upload (ROS only)
#
# Override paths/port via environment variables (see below).

set -e

REPO="${REPO:-$HOME/kiro}"
WS="${WS:-$HOME/ros2_ws}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
FQBN="${FQBN:-arduino:avr:uno}"
PKG="qr_coordinates_ros2"

echo "==> Pulling latest code in ${REPO}…"
git -C "${REPO}" pull origin main

echo "==> Refreshing the package in the workspace…"
rm -rf "${WS}/src/${PKG}"
cp -r "${REPO}/${PKG}" "${WS}/src/"

echo "==> Building ${PKG}…"
cd "${WS}"
source /opt/ros/humble/setup.bash
colcon build --packages-select "${PKG}"
source "${WS}/install/setup.bash"
echo "    ROS build done."

if [ "$1" != "--no-flash" ]; then
  echo "==> Compiling + uploading the Arduino sketch to ${SERIAL_PORT}…"
  cd "${REPO}/${PKG}/arduino"
  arduino-cli compile --fqbn "${FQBN}" agv_xy_align
  arduino-cli upload  --fqbn "${FQBN}" -p "${SERIAL_PORT}" agv_xy_align
  echo "    Arduino flashed."
else
  echo "==> Skipping Arduino flash (--no-flash)."
fi

echo
echo "Done. Now run:  ./run.sh"
