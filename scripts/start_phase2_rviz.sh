#!/usr/bin/env bash
# Phase 2 (RViz path): Phase 0+1 perception stack + MoveIt demo + moveit_servo + executor.
# Run inside Docker. Requires DISPLAY for Gazebo GUI and RViz.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"

RUN_SMOKE="${1:-}"
source_ros
stop_stack
configure_gl_env

# --- Phase 0+1: Gazebo camera + perception ---
_start_bg gazebo_desk ros2 launch panda_sim_bringup gazebo_desk_only.launch.py
echo "==> Gazebo desk starting (18s)"
sleep 18
wait_for_topic /camera/color/image_raw 30

_start_bg static_tf ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id panda_link0

# --- Phase 2: MoveIt + servo + pick/place ---
_start_bg moveit_demo ros2 launch my_panda_moveit_config demo.launch.py
echo "==> MoveIt demo starting (20s)"
sleep 20
wait_for_action move_action 90

_start_bg moveit_servo ros2 launch panda_pick_place phase2_rviz.launch.py
echo "==> Phase 2 nodes starting (10s)"
sleep 10
wait_for_service /perception/trigger_scan 30
wait_for_action /pick_place/pick_object 30

echo ""
echo "Phase 2 (RViz) stack is up."
echo "  move_action, moveit_servo, pick_place ready"
echo "  allow_gripper_skip=true (RViz has no franka_gripper)"
echo "  Logs: $LOG_DIR"
echo "  Verify: bash $ROOT/scripts/verify_phase2.sh"
echo "  Smoke:  bash $ROOT/scripts/smoke_pick_place.sh"
echo "  Stop:   bash $ROOT/scripts/stop_stack.sh"

if [[ "$RUN_SMOKE" == "--smoke" ]]; then
  bash "$ROOT/scripts/verify_phase0.sh"
  bash "$ROOT/scripts/verify_perception.sh"
  bash "$ROOT/scripts/verify_phase2.sh"
  bash "$ROOT/scripts/smoke_pick_place.sh" || true
fi
