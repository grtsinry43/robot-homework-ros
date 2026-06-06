#!/usr/bin/env bash
# Restart desk-only Gazebo (use inside container). Keeps host DISPLAY for GUI.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"
source_ros

pkill -f "ign gazebo.*pick_place_desk" 2>/dev/null || true
sleep 2
configure_gl_env
ros2 launch panda_sim_bringup gazebo_desk_only.launch.py
