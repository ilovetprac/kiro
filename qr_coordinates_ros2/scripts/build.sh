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

# Auto-detect the ROS 2 distro so this works on both Jazzy (24.04) and
# Humble (22.04). Override with ROS_DISTRO=<name> if needed.
detect_ros_distro() {
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "${ROS_DISTRO}"; return
  fi
  for d in jazzy humble iron rolling; do
    if [ -f "/opt/ros/${d}/setup.bash" ]; then echo "${d}"; return; fi
  done
  echo ""   # none found
}
ROS="$(detect_ros_distro)"
if [ -z "${ROS}" ]; then
  echo "ERROR: no ROS 2 distro found in /opt/ros/. Install ROS 2 first." >&2
  exit 1
fi

echo "==> Pulling latest code in ${REPO}…"
git -C "${REPO}" pull origin main

echo "==> Refreshing the package in the workspace…"
rm -rf "${WS}/src/${PKG}"
cp -r "${REPO}/${PKG}" "${WS}/src/"

echo "==> Building ${PKG} (ROS ${ROS})…"
cd "${WS}"
source "/opt/ros/${ROS}/setup.bash"
colcon build --packages-select "${PKG}"
source "${WS}/install/setup.bash"

echo
echo "Done. Now run:  ./run.sh"
