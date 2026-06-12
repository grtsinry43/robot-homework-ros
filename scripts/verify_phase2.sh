#!/usr/bin/env bash
# Phase 2 readiness: MoveIt move_action + pick_place actions + optional servo.
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

echo "==> MoveIt + inner loop"
check "move_action" bash -c 'ros2 action list | grep -q move_action'
check "pick_object action" bash -c 'ros2 action list | grep -q /pick_place/pick_object'
check "place_at action" bash -c 'ros2 action list | grep -q /pick_place/place_at'
check "executor node" bash -c 'ros2 node list | grep -q pick_place_executor'
check "servo_node" bash -c 'ros2 node list | grep -q servo_node'
check "panda_arm_controller active" bash -c \
  'ros2 control list_controllers 2>/dev/null | grep -q "panda_arm_controller.*active"'
check "panda_gripper_controller active" bash -c \
  'ros2 control list_controllers 2>/dev/null | grep -q "panda_gripper_controller.*active"'
check "franka_gripper move" bash -c 'ros2 action list | grep -q /franka_gripper/move'
check "franka_gripper grasp" bash -c 'ros2 action list | grep -q /franka_gripper/grasp'
check "gazebo_gripper_sim" bash -c 'ros2 node list | grep -q gazebo_gripper_sim'

echo "==> Summary: $PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
