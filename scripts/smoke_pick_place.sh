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
  PICK_ID=$(_ids_from_scene | grep -E 'red_block_01|green_block_01' | head -1 || true)
  [[ -z "$PICK_ID" ]] && PICK_ID=$(_ids_from_scene | grep -E 'red_block|green_block' | head -1 || true)
  [[ -z "$PICK_ID" ]] && PICK_ID=$(_ids_from_scene | head -1)
fi
if [[ -z "$PLACE_ID" ]]; then
  PLACE_ID=$(_ids_from_scene | grep -E 'blue_plate_01' | head -1 || true)
  [[ -z "$PLACE_ID" ]] && PLACE_ID=$(_ids_from_scene | grep -E 'blue_plate' | head -1 || true)
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

_send_goal_expect_success() {
  local label="$1"
  local timeout_sec="$2"
  local action_name="$3"
  local action_type="$4"
  local goal_yaml="$5"
  local output
  local rc

  set +e
  output="$(timeout "$timeout_sec" ros2 action send_goal "$action_name" "$action_type" "$goal_yaml" --feedback 2>&1)"
  rc=$?
  set -e

  echo "$output"
  if [[ "$rc" -eq 124 ]]; then
    echo "[FAIL] $label timed out" >&2
    exit 1
  fi
  if [[ "$rc" -ne 0 ]]; then
    echo "[FAIL] $label exited with code $rc" >&2
    exit 1
  fi
  if ! grep -q "success: true" <<<"$output"; then
    local status
    local code
    local reason
    status="$(printf '%s\n' "$output" | sed -n 's/^Goal finished with status: //p' | tail -1)"
    code="$(printf '%s\n' "$output" | sed -n 's/^[[:space:]]*code:[[:space:]]*//p' | tail -1 | tr -d "'")"
    reason="$(printf '%s\n' "$output" | sed -n 's/^[[:space:]]*reason:[[:space:]]*//p' | tail -1 | tr -d "'")"
    echo "[FAIL] $label result not successful${status:+ status=$status}${code:+ code=$code}${reason:+ reason=$reason}" >&2
    exit 1
  fi
}

echo "==> Pick $PICK_ID (timeout ${PICK_TIMEOUT}s)"
_send_goal_expect_success \
  "pick" "$PICK_TIMEOUT" \
  /pick_place/pick_object pick_place_msgs/action/PickObject \
  "{object_id: '${PICK_ID}'}"

echo "==> Refresh scene before place"
ros2 service call /perception/trigger_scan pick_place_msgs/srv/TriggerScan "{}" || true
sleep 1

echo "==> Place on $PLACE_ID (timeout ${PLACE_TIMEOUT}s)"
_send_goal_expect_success \
  "place" "$PLACE_TIMEOUT" \
  /pick_place/place_at pick_place_msgs/action/PlaceAt \
  "{target_id: '${PLACE_ID}', offset: 'above'}"

echo "[PASS] Smoke test done (pick + place)"
