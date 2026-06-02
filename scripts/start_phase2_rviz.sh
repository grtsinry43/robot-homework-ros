#!/usr/bin/env bash
# Phase 2: 同一 Gazebo 场景 — 桌面/物体 + Panda 臂（gz_ros2_control）+ MoveIt 内环。
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"

RUN_SMOKE="${1:-}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-0}"

source_ros
stop_stack
configure_gl_env

# --- 唯一仿真场景：Panda + 桌面 + 相机（MoveIt 轨迹在同一 Gazebo 里执行）---
_start_bg gazebo_panda ros2 launch panda_sim_bringup gazebo_panda_pick_place.launch.py
# log: $LOG_DIR/gazebo_panda.log
echo "==> Gazebo Panda + desk starting (50s)"
sleep 50
wait_for_topic /camera/color/image_raw 60 || echo "WARN: camera topics missing"
wait_for_arm_controller 120

# world→panda_link0 已由 URDF 固定关节 + robot_state_publisher 发布，无需额外 static TF

# --- MoveIt（不再启动 mock spawn_controllers；控制器在 Gazebo 内）---
_start_bg moveit_grp ros2 launch my_panda_moveit_config move_group.launch.py
sleep 8
wait_for_action move_action 90

if [[ "$LAUNCH_RVIZ" == "1" ]]; then
  echo "==> Optional RViz (仅调试规划，真机在 Gazebo)"
  _start_bg moveit_rviz ros2 launch my_panda_moveit_config moveit_rviz.launch.py
  sleep 5
fi

_start_bg moveit_servo ros2 launch panda_pick_place moveit_servo.launch.py
_start_bg pick_place ros2 launch panda_pick_place phase2_rviz.launch.py
sleep 10
wait_for_service /perception/trigger_scan 30
wait_for_action /pick_place/pick_object 30

echo ""
echo "Phase 2 stack: ONE Gazebo scene (arm + objects + camera)."
echo "  只看 Gazebo 窗口 — 机械臂与物体在同一仿真里"
echo "  MoveIt 通过 panda_arm_controller 驱动 Gazebo 中的 Panda"
echo "  调试 RViz: LAUNCH_RVIZ=1 $0"
echo "  Logs: $LOG_DIR"
echo "  Verify/Smoke: verify_perception.sh / verify_phase2.sh / smoke_pick_place.sh"
echo "  Stop: stop_stack.sh"

if [[ "$RUN_SMOKE" == "--smoke" ]]; then
  bash "$ROOT/scripts/verify_phase0.sh"
  bash "$ROOT/scripts/verify_perception.sh"
  bash "$ROOT/scripts/verify_phase2.sh"
  bash "$ROOT/scripts/smoke_pick_place.sh" || true
fi
