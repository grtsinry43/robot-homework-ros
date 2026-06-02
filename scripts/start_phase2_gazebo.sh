#!/usr/bin/env bash
# Phase 2 (Gazebo + Franka): vendor Franka sim + desk camera bridge.
# Desk world is passed to Franka bringup via gz_args (pick_place_desk.sdf).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"

source_ros
stop_stack
configure_gl_env

_start_bg gazebo_franka ros2 launch panda_sim_bringup gazebo_pick_place.launch.py load_gripper:=true
echo "==> Franka Gazebo starting (45s — first run may take longer)"
sleep 45
wait_for_topic /camera/color/image_raw 60 || echo "WARN: camera topics missing (desk world may not be merged yet)"

_start_bg static_tf ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id panda_link0

_start_bg moveit_servo ros2 launch panda_pick_place moveit_servo.launch.py
_start_bg pick_place ros2 launch panda_pick_place pick_place.launch.py

echo ""
echo "Phase 2 (Gazebo+Franka) stack started (experimental)."
echo "  If arm TF differs from static world->panda_link0, fix world merge or hand-eye TF."
echo "  Logs: $LOG_DIR"
echo "  Stop: bash $ROOT/scripts/stop_stack.sh"
