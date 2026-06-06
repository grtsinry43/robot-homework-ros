#!/usr/bin/env bash
# Full verification of implemented stack.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"
source_ros

cleanup() {
  bash "$ROOT/scripts/stop_stack.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "==> Starting Phase 0+1 stack"
bash "$ROOT/scripts/start_phase01.sh"

bash "$ROOT/scripts/verify_phase0.sh"
bash "$ROOT/scripts/verify_perception.sh"

echo "==> executor/action checks"
ros2 node list | grep -q pick_place_executor && echo "[PASS] pick_place_executor node"
ros2 action list | grep -q /pick_place/pick_object && echo "[PASS] pick_object action"
ros2 action list | grep -q /pick_place/place_at && echo "[PASS] place_at action"

echo "==> ALL CHECKS DONE"
