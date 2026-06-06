#!/usr/bin/env bash
# Smoke test pick-place stack via ros2 CLI (no MCP client needed).
# Prerequisite: start_phase01.sh or start_phase2_rviz.sh, workspace sourced.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"
source_ros

echo "==> Waiting for /scene_state..."
timeout 15 bash -c 'until ros2 topic list | grep -q /scene_state; do sleep 0.5; done'

echo "==> Trigger scan (up to 3 passes)"
for _ in 1 2 3; do
  ros2 service call /perception/trigger_scan pick_place_msgs/srv/TriggerScan "{}" || true
  sleep 0.8
done

echo "==> Scene snapshot"
SCENE=$(timeout 8 ros2 topic echo /scene_state --once 2>&1 || true)
echo "$SCENE"

PICK_ID="${SMOKE_PICK_ID:-}"
PLACE_ID="${SMOKE_PLACE_ID:-}"
# grep returns 1 when no match — must not trip `set -o pipefail`
_ids_from_scene() {
  echo "$SCENE" | sed -n 's/^[[:space:]]*- id: \([^[:space:]]*\).*/\1/p'
}
if [[ -z "$PICK_ID" ]]; then
  PICK_ID=$(_ids_from_scene | grep -E 'red_block|green_block' | head -1 || true)
  [[ -z "$PICK_ID" ]] && PICK_ID=$(_ids_from_scene | head -1)
fi
if [[ -z "$PLACE_ID" ]]; then
  PLACE_ID=$(_ids_from_scene | grep -E 'blue_plate' | head -1 || true)
  if [[ -z "$PLACE_ID" ]]; then
    PLACE_ID=$(_ids_from_scene | head -1 | sed 's/red_block/blue_plate/; s/green_block/blue_plate/')
  fi
  [[ -z "$PLACE_ID" ]] && PLACE_ID="blue_plate_01"
fi
if [[ -z "$PICK_ID" ]]; then
  echo "ERROR: no object id in /scene_state; run perception with Gazebo desk up" >&2
  exit 1
fi

PICK_TIMEOUT="${SMOKE_PICK_TIMEOUT:-180}"
PLACE_TIMEOUT="${SMOKE_PLACE_TIMEOUT:-180}"

echo "==> Pick $PICK_ID (timeout ${PICK_TIMEOUT}s)"
timeout "$PICK_TIMEOUT" ros2 action send_goal /pick_place/pick_object pick_place_msgs/action/PickObject "{object_id: '${PICK_ID}'}" --feedback
PICK_RC=$?
if [[ "$PICK_RC" -eq 124 ]]; then
  echo "[FAIL] pick timed out" >&2
  exit 1
fi
if [[ "$PICK_RC" -ne 0 ]]; then
  echo "[FAIL] pick exited with code $PICK_RC" >&2
  exit 1
fi

echo "==> Place on $PLACE_ID (timeout ${PLACE_TIMEOUT}s)"
timeout "$PLACE_TIMEOUT" ros2 action send_goal /pick_place/place_at pick_place_msgs/action/PlaceAt "{target_id: '${PLACE_ID}', offset: 'above'}" --feedback
PLACE_RC=$?
if [[ "$PLACE_RC" -eq 124 ]]; then
  echo "[FAIL] place timed out" >&2
  exit 1
fi
if [[ "$PLACE_RC" -ne 0 ]]; then
  echo "[FAIL] place exited with code $PLACE_RC" >&2
  exit 1
fi

echo "[PASS] Smoke test done (pick + place)"
