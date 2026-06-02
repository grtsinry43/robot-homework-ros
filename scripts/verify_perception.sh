#!/usr/bin/env bash
# Perception + /scene_state verification (DESIGN.md §5.2–§5.3).
# Prerequisite: gazebo_desk_only + static world->panda_link0 + pick_place.launch.py
set -eo pipefail

source /opt/ros/humble/setup.bash
[[ -f /root/ros2_ws/install/setup.bash ]] && source /root/ros2_ws/install/setup.bash

echo "==> wait for perception service"
for _ in $(seq 1 20); do
  if ros2 service list 2>/dev/null | grep -q '/perception/trigger_scan'; then
    break
  fi
  sleep 0.5
done

echo "==> trigger_scan"
ros2 service call /perception/trigger_scan pick_place_msgs/srv/TriggerScan "{}"

echo "==> scene_state (once)"
OUT=$(timeout 8 ros2 topic echo /scene_state --once 2>&1 || true)
echo "$OUT"
echo "$OUT" | grep -q 'id:' && echo "[PASS] scene_state has objects" || { echo "[FAIL] no objects in scene_state"; exit 1; }
