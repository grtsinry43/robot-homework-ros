#!/usr/bin/env bash
# Full verification of implemented stack (run inside container).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"
source_ros

echo "==> Starting Gazebo desk (DISPLAY=$DISPLAY)"
configure_gl_env
ros2 launch panda_sim_bringup gazebo_desk_only.launch.py > /tmp/verify_gz.log 2>&1 &
GZ_PID=$!
sleep 18

cleanup() {
  kill "$GZ_PID" 2>/dev/null || true
  pkill -f perception_node 2>/dev/null || true
  pkill -f static_transform_publisher 2>/dev/null || true
}
trap cleanup EXIT

bash /root/scripts/verify_phase0.sh

echo "==> Static TF + perception"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world panda_link0 &
TF_PID=$!
sleep 1
ros2 run panda_pick_place perception_node \
  --ros-args --params-file /root/ros2_ws/install/panda_pick_place/share/panda_pick_place/config/pick_place_params.yaml &
PERC_PID=$!
sleep 3

bash /root/scripts/verify_perception.sh
kill "$PERC_PID" "$TF_PID" 2>/dev/null || true
wait "$PERC_PID" 2>/dev/null || true

echo "==> executor_node import/start"
timeout 3 ros2 run panda_pick_place executor_node \
  --ros-args --params-file /root/ros2_ws/install/panda_pick_place/share/panda_pick_place/config/pick_place_params.yaml 2>&1 | grep -q "pick_place_executor ready" && echo "[PASS] executor_node starts"

echo "==> ALL CHECKS DONE"
