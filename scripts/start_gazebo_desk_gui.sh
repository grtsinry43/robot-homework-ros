#!/usr/bin/env bash
# Start ONLY the desk Gazebo world (one instance). Use before opening any other sim.
# Run inside container; on host: ./scripts/run_in_container.sh start-gazebo-desk
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"

source_ros
stop_stack
configure_gl_env

echo "==> Tip: close old Gazebo windows, then watch the NEW window from this launch."
_start_bg gazebo_desk ros2 launch panda_sim_bringup gazebo_desk_only.launch.py
sleep 20
wait_for_topic /camera/color/image_raw 25 || true

echo ""
echo "Gazebo desk running. Entity tree should list: work_table, red_block_01, blue_plate_01, ..."
echo "If viewport is empty: Gazebo menu -> View -> Reset View / press 'r' in 3D view."
echo "Logs: $LOG_DIR/gazebo_desk.log"
