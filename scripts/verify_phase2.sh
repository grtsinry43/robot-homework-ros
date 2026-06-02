#!/usr/bin/env bash
# Phase 2 readiness: MoveIt move_action + pick_place actions + optional servo.
set -eo pipefail

source /opt/ros/humble/setup.bash
[[ -f /root/ros2_ws/install/setup.bash ]] && source /root/ros2_ws/install/setup.bash

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

echo "==> Summary: $PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
