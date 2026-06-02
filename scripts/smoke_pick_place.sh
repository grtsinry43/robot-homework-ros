#!/usr/bin/env bash
# Smoke test pick-place stack via ros2 CLI (no MCP client needed).
# Prerequisite: start_phase01.sh or start_phase2_rviz.sh, workspace sourced.
set -eo pipefail

source /opt/ros/humble/setup.bash
[[ -f /root/ros2_ws/install/setup.bash ]] && source /root/ros2_ws/install/setup.bash

echo "==> Waiting for /scene_state..."
timeout 15 bash -c 'until ros2 topic list | grep -q /scene_state; do sleep 0.5; done'

echo "==> Trigger scan"
ros2 service call /perception/trigger_scan pick_place_msgs/srv/TriggerScan "{}"

echo "==> Scene snapshot"
SCENE=$(timeout 8 ros2 topic echo /scene_state --once 2>&1 || true)
echo "$SCENE"

PICK_ID="${SMOKE_PICK_ID:-}"
PLACE_ID="${SMOKE_PLACE_ID:-}"
if [[ -z "$PICK_ID" ]]; then
  PICK_ID=$(echo "$SCENE" | sed -n 's/.*\bid: \([^[:space:]]*\).*/\1/p' | head -1)
fi
if [[ -z "$PLACE_ID" ]]; then
  PLACE_ID=$(echo "$SCENE" | sed -n 's/.*\bid: \([^[:space:]]*\).*/\1/p' | head -1 \
    | sed 's/red_block/blue_plate/; s/green_block/blue_plate/')
  [[ -z "$PLACE_ID" ]] && PLACE_ID="blue_plate_01"
fi
if [[ -z "$PICK_ID" ]]; then
  echo "ERROR: no object id in /scene_state; run perception with Gazebo desk up" >&2
  exit 1
fi

echo "==> Pick $PICK_ID"
timeout 120 ros2 action send_goal /pick_place/pick_object pick_place_msgs/action/PickObject "{object_id: $PICK_ID}" --feedback

echo "==> Place on $PLACE_ID"
timeout 120 ros2 action send_goal /pick_place/place_at pick_place_msgs/action/PlaceAt "{target_id: $PLACE_ID, offset: above}" --feedback

echo "==> Smoke test done"
