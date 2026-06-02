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
if ! echo "$OUT" | grep -q 'id:'; then
  echo "[FAIL] no objects in scene_state"
  exit 1
fi
echo "[PASS] scene_state has objects"
N_OBJ=$(echo "$OUT" | grep -c '^[[:space:]]*- id:' || true)
echo "==> object count: $N_OBJ"
if [[ "$N_OBJ" -ge 2 ]]; then
  echo "[PASS] multi-object detection (>=2)"
elif [[ "$N_OBJ" -eq 1 ]]; then
  echo "[WARN] only 1 object — HSV may need tuning for red/green blocks"
else
  echo "[FAIL] no object ids"
  exit 1
fi

# Desk layout in panda_link0 (pick_place_desk.sdf): x≈0.3–0.55, y≈±0.12, z≈0.05–0.08
POS=$(echo "$OUT" | awk '/position:/{getline; if ($1=="x:") {x=$2; getline; y=$2; getline; z=$2; print x,y,z; exit}}')
if [[ -n "$POS" ]]; then
  read -r PX PY PZ <<< "$POS"
  echo "==> first object position: $PX $PY $PZ"
  python3 - <<PY
px, py, pz = float("$PX"), float("$PY"), float("$PZ")
ok = 0.25 <= px <= 0.65 and -0.25 <= py <= 0.25 and 0.0 <= pz <= 0.20
print("[PASS] pose on desk envelope" if ok else "[FAIL] pose off desk (check camera TF / frame_id)")
raise SystemExit(0 if ok else 1)
PY
else
  echo "[WARN] could not parse position for desk check"
fi
