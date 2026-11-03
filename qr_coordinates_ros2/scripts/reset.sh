#!/usr/bin/env bash
#
# reset.sh — tell the running gpio_stepper_node to return to the power-on
# origin (0,0) and exit cleanly.
#
# Run this in a SECOND terminal while the system (run.sh / agv_align.launch.py)
# is running. It publishes a one-shot message to the node's ~/reset topic.
#
# Usage:
#   ./reset.sh
#
# (Equivalent manual command:
#    ros2 topic pub --once /gpio_stepper_node/reset std_msgs/msg/Empty "{}")

set -e

WS="${WS:-$HOME/ros2_ws}"
NODE="${NODE:-/gpio_stepper_node}"

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

echo "==> Sending RESET (return to origin + quit) to ${NODE}…"
ros2 topic pub --once "${NODE}/reset" std_msgs/msg/Empty "{}"
echo "==> Done. The stepper node will home to (0,0) and exit."
