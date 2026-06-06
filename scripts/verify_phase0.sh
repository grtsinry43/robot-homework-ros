#!/usr/bin/env bash
# Phase 0 verification (DESIGN.md §7, CLAUDE.md checklist).
# Run inside container after: source install/setup.bash
# Prerequisite: gazebo_desk_only.launch.py already running.
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

echo "==> 1. Camera topics"
check "color image topic" bash -c "ros2 topic list | grep -q '/camera/color/image_raw'"
check "depth image topic" bash -c "ros2 topic list | grep -q '/camera/depth/image_raw'"

echo "==> 2. Camera publish rate"
RATE=$(timeout 10 ros2 topic hz /camera/color/image_raw 2>&1 | awk '/average rate/ {print $3; exit}' || true)
if [[ -n "${RATE:-}" ]] && awk "BEGIN {exit !($RATE > 5)}"; then
  echo "[PASS] color image hz ($RATE)"
  PASS=$((PASS + 1))
else
  echo "[FAIL] color image hz (got: ${RATE:-none})"
  FAIL=$((FAIL + 1))
fi

echo "==> 3. TF world -> camera_optical_frame"
check "tf2_echo" bash -c "timeout 5 ros2 run tf2_ros tf2_echo world camera_optical_frame 2>&1 | grep -q Translation"

echo "==> Summary: $PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
