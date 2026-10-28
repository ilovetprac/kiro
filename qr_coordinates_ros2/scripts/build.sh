#!/usr/bin/env bash
#
# build.sh — pull latest code and rebuild the ROS 2 package. Run this once
# after pulling new changes; then use run.sh for day-to-day startup.
#
# Usage:
#   ./build.sh
#
# Override paths via environment variables (REPO, WS).

set -e

REPO="${REPO:-$HOME/kiro}"
WS="${WS:-$HOME/ros2_ws}"
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

echo
echo "Done. Now run:  ./run.sh"
