#!/usr/bin/env bash
#
# build_librealsense_rsusb.sh
# ---------------------------
# Build Intel librealsense from source with the RSUSB (libusb) backend on a
# Raspberry Pi 4B / Ubuntu 22.04.
#
# WHY: On the Pi, the kernel UVC driver (e.g. kernel 5.15.0-1110-raspi) is often
# NEWER than the versions librealsense's V4L2 backend is patched for. That makes
# UVC extension-unit control transfers fail with:
#     xioctl(UVCIOC_CTRL_QUERY) failed on control 1 Last Error: Connection timed out
# The RSUSB backend talks to the camera over libusb directly, bypassing the
# kernel UVC layer entirely — which eliminates that error.
#
# This script:
#   1. installs build dependencies,
#   2. adds temporary swap so the compile doesn't OOM on a 4GB Pi,
#   3. builds + installs librealsense with FORCE_RSUSB_BACKEND=true,
#   4. installs the udev rules,
#   5. removes the temporary swap.
#
# After it finishes, TEST WITHOUT ROS first:
#     rs-enumerate-devices
#     realsense-viewer        # enable Depth + Color; confirm no XU timeouts
#
# Usage:
#     chmod +x build_librealsense_rsusb.sh
#     ./build_librealsense_rsusb.sh
#
set -euo pipefail

SWAPFILE=/swapfile_rs
LIBRS_DIR="${HOME}/librealsense"
JOBS=2   # 4GB Pi: keep this at 2 to avoid the compiler getting OOM-killed.

echo "==> [1/6] Installing build dependencies (apt)…"
sudo apt update
sudo apt install -y \
  git cmake build-essential pkg-config \
  libssl-dev libusb-1.0-0-dev libudev-dev \
  libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
  libgtk-3-dev \
  python3-dev

echo "==> [2/6] Adding ${SWAPFILE} (2G) so the build does not run out of RAM…"
if [ ! -f "${SWAPFILE}" ]; then
  sudo fallocate -l 2G "${SWAPFILE}"
  sudo chmod 600 "${SWAPFILE}"
  sudo mkswap "${SWAPFILE}"
  sudo swapon "${SWAPFILE}"
  echo "    temporary swap enabled."
else
  echo "    ${SWAPFILE} already exists; skipping."
fi

echo "==> [3/6] Fetching librealsense source…"
if [ ! -d "${LIBRS_DIR}" ]; then
  git clone https://github.com/IntelRealSense/librealsense.git "${LIBRS_DIR}"
fi
cd "${LIBRS_DIR}"
# Use a released tag for reproducibility; comment out to build the default branch.
# git checkout v2.56.3

echo "==> [4/6] Installing udev rules (camera accessible without sudo)…"
sudo ./scripts/setup_udev_rules.sh || true

echo "==> [5/6] Building librealsense with the RSUSB backend (this takes a while)…"
rm -rf build
mkdir build && cd build
cmake .. \
  -DFORCE_RSUSB_BACKEND=true \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true \
  -DBUILD_GRAPHICAL_EXAMPLES=true \
  -DBUILD_PYTHON_BINDINGS=false
make -j"${JOBS}"
sudo make install
sudo ldconfig

echo "==> [6/6] Removing temporary swap…"
if swapon --show | grep -q "${SWAPFILE}"; then
  sudo swapoff "${SWAPFILE}" || true
fi
sudo rm -f "${SWAPFILE}"

echo
echo "============================================================"
echo "Done. Now UNPLUG and REPLUG the D435, then test WITHOUT ROS:"
echo "    rs-enumerate-devices"
echo "    realsense-viewer      # enable Depth + Color"
echo
echo "If the viewer streams depth+color with no UVCIOC_CTRL_QUERY"
echo "timeouts, rebuild realsense-ros against THIS librealsense,"
echo "then re-run the ROS launch."
echo "============================================================"
