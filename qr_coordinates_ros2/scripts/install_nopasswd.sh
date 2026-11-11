#!/usr/bin/env bash
#
# install_nopasswd.sh — one-time setup so run.sh can free the RealSense camera
# WITHOUT asking for a sudo password every time.
#
# It installs a narrowly-scoped sudoers rule (only the uvcvideo/fuser/usbreset/
# authorized-toggle commands run.sh needs) to /etc/sudoers.d/. Run this ONCE.
#
# Usage:
#   ./install_nopasswd.sh          # sets it up for the current user
#   sudo ./install_nopasswd.sh     # also fine
#
# You will be asked for your password ONE time (to install the rule). After
# that, ./run.sh never prompts again.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/99-realsense-nopasswd.sudoers"
DEST="/etc/sudoers.d/99-realsense-nopasswd"

# The user run.sh is used by (the one invoking this, even under sudo).
TARGET_USER="${SUDO_USER:-$USER}"

if [ ! -f "${TEMPLATE}" ]; then
  echo "ERROR: template not found: ${TEMPLATE}" >&2
  exit 1
fi

echo "==> Installing password-free camera-freeing rule for user: ${TARGET_USER}"

# Build the rule with the real username substituted, into a temp file.
TMP="$(mktemp)"
trap 'rm -f "${TMP}"' EXIT
sed "s/__USER__/${TARGET_USER}/g" "${TEMPLATE}" > "${TMP}"

# Validate the sudoers syntax BEFORE installing (never install a broken file —
# a malformed sudoers can lock you out of sudo).
if ! sudo visudo -cf "${TMP}" >/dev/null; then
  echo "ERROR: generated sudoers file failed validation; not installing." >&2
  exit 1
fi

# Install with the correct owner/permissions (sudoers.d requires 0440 root).
sudo install -o root -g root -m 0440 "${TMP}" "${DEST}"

# Final sanity check of the whole sudoers configuration.
if sudo visudo -c >/dev/null; then
  echo "==> Installed ${DEST}"
  echo "==> Done. ./run.sh will no longer prompt for a password to free the camera."
else
  echo "WARNING: sudoers validation reported an issue — check manually:" >&2
  echo "         sudo visudo -c" >&2
  exit 1
fi
