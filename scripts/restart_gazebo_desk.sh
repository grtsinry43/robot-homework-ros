#!/usr/bin/env bash
# Restart desk-only Gazebo (use inside container). Keeps host DISPLAY for GUI.
set -eo pipefail
source /opt/ros/humble/setup.bash
[[ -f /root/ros2_ws/install/setup.bash ]] && source /root/ros2_ws/install/setup.bash

pkill -f "ign gazebo.*pick_place_desk" 2>/dev/null || true
sleep 2
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
ros2 launch panda_sim_bringup gazebo_desk_only.launch.py
