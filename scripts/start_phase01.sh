#!/usr/bin/env bash
# Phase 0+1: Gazebo desk + camera + static world->panda_link0 + perception/executor.
# Run inside Docker after: source install/setup.bash
# Host: scripts/run_in_container.sh start-phase01
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"

source_ros
stop_stack
configure_gl_env

_is_wsl() {
  grep -qiE "(microsoft|wsl)" /proc/version 2>/dev/null
}

_phase01_use_gui="${PHASE01_USE_GUI:-auto}"
case "${_phase01_use_gui,,}" in
  auto)
    if [[ -n "${DISPLAY:-}" ]] && ! _is_wsl; then
      _phase01_use_gui="true"
    else
      _phase01_use_gui="false"
    fi
    ;;
  1|true|yes|on)
    _phase01_use_gui="true"
    ;;
  0|false|no|off)
    _phase01_use_gui="false"
    ;;
  *)
    echo "ERROR: PHASE01_USE_GUI must be auto|true|false (got: ${PHASE01_USE_GUI})" >&2
    exit 1
    ;;
esac
echo "==> Gazebo GUI client: ${_phase01_use_gui} (PHASE01_USE_GUI=${PHASE01_USE_GUI:-auto})"

_start_bg gazebo_desk ros2 launch panda_sim_bringup gazebo_desk_only.launch.py use_gui:="${_phase01_use_gui}"
echo "==> waiting for Gazebo + camera (18s)"
sleep 18
wait_for_topic /camera/color/image_raw 30

_start_bg static_tf ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id panda_link0

_start_bg pick_place ros2 launch panda_pick_place pick_place.launch.py

echo "==> waiting for perception (8s)"
sleep 8
wait_for_service /perception/trigger_scan 30

echo ""
echo "Phase 0+1 stack is up."
echo "  Logs: $LOG_DIR"
echo "  PIDs: $PID_DIR"
echo "  Verify: bash $ROOT/scripts/verify_phase0.sh"
echo "          bash $ROOT/scripts/verify_perception.sh"
echo "  Stop:   bash $ROOT/scripts/stop_stack.sh"
