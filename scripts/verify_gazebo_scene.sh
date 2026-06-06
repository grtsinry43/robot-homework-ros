#!/usr/bin/env bash
# Verify single Gazebo scene: Panda in sim + camera + arm controller active.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"
source_ros

PASS=0
FAIL=0
check() {
  local name="$1"
  shift
  if "$@"; then
    echo "[PASS] $name"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] $name"
    FAIL=$((FAIL + 1))
  fi
}

echo "==> Gazebo scene checks"
check "camera topic" bash -c 'ros2 topic list | grep -q /camera/color/image_raw'
check "joint_states" bash -c 'ros2 topic list | grep -q /joint_states'
check "panda_arm_controller active" bash -c \
  'ros2 control list_controllers 2>/dev/null | grep -q "panda_arm_controller.*active"'
check "move_action" bash -c 'ros2 action list | grep -q move_action'
check "pick_place action" bash -c 'ros2 topic list | grep -q /scene_state'

echo "==> joint names (expect panda_joint*)"
timeout 3 ros2 topic echo /joint_states --once 2>/dev/null | grep -E '^- ' | head -8 || true

echo "==> Summary: $PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
