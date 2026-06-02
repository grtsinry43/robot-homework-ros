# Shared helpers for stack start/stop scripts (source, do not execute).
set -eo pipefail

export ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
export WS_SETUP="${WS_SETUP:-/root/ros2_ws/install/setup.bash}"
export PID_DIR="${PID_DIR:-/tmp/robot_homework_ros}"
export LOG_DIR="${LOG_DIR:-/tmp/robot_homework_ros/logs}"

source_ros() {
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
  if [[ -f "$WS_SETUP" ]]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
  else
    echo "ERROR: workspace not built: $WS_SETUP missing" >&2
    return 1
  fi
}

mkdir -p "$PID_DIR" "$LOG_DIR"

_start_bg() {
  local name="$1"
  shift
  local log="$LOG_DIR/${name}.log"
  echo "==> starting $name (log: $log)"
  # shellcheck disable=SC2090
  nohup "$@" >"$log" 2>&1 &
  echo $! >"$PID_DIR/${name}.pid"
  echo "    pid $(cat "$PID_DIR/${name}.pid")"
}

_stop_one() {
  local name="$1"
  local pidfile="$PID_DIR/${name}.pid"
  if [[ -f "$pidfile" ]]; then
    local pid
    pid="$(cat "$pidfile")"
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      sleep 1
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$pidfile"
  fi
  pkill -f "$name" 2>/dev/null || true
}

stop_stack() {
  echo "==> stopping stack"
  for name in pick_place moveit_servo moveit_demo moveit_grp moveit_ctrl moveit_rsp moveit_rviz static_tf gazebo_desk gazebo_franka gazebo_unified gazebo_panda; do
    _stop_one "$name"
  done
  pkill -f "ign gazebo" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  killall -9 ign 2>/dev/null || true
  pkill -9 -f perception_node 2>/dev/null || true
  pkill -9 -f executor_node 2>/dev/null || true
  pkill -9 -f move_group 2>/dev/null || true
  pkill -9 -f ros2_control_node 2>/dev/null || true
  pkill -9 -f rviz2 2>/dev/null || true
  pkill -9 -f servo_node 2>/dev/null || true
  pkill -9 -f spawner 2>/dev/null || true
  pkill -9 -f "ros2 control" 2>/dev/null || true
  pkill -f "ros2 launch panda_pick_place" 2>/dev/null || true
  pkill -f "ros2 launch my_panda_moveit_config" 2>/dev/null || true
  pkill -f "ros2 launch panda_sim_bringup" 2>/dev/null || true
  pkill -f static_transform_publisher 2>/dev/null || true
  pkill -f overhead_camera_tf 2>/dev/null || true
  pkill -f camera_gz_rgbd_alias 2>/dev/null || true
  pkill -f robot_state_publisher 2>/dev/null || true
  pkill -f ros_gz_camera_bridge 2>/dev/null || true
  sleep 3
  true
}

# Use host GPU rendering when X11 is available; software GL breaks the Gazebo GUI.
configure_gl_env() {
  if [[ -n "${DISPLAY:-}" ]]; then
    unset LIBGL_ALWAYS_SOFTWARE
    unset MESA_LOADER_DRIVER_OVERRIDE
    unset GALLIUM_DRIVER
    echo "==> DISPLAY=$DISPLAY — using host GPU for Gazebo GUI"
  else
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
    echo "==> no DISPLAY — headless/software GL"
  fi
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-30}"
  echo "==> wait topic $topic (${timeout}s)"
  timeout "$timeout" bash -c "source \"$ROS_SETUP\"; [[ -f \"$WS_SETUP\" ]] && source \"$WS_SETUP\"; until ros2 topic list 2>/dev/null | grep -q \"${topic}\"; do sleep 0.5; done"
}

wait_for_action() {
  local action="$1"
  local timeout="${2:-60}"
  echo "==> wait action $action (${timeout}s)"
  timeout "$timeout" bash -c "source \"$ROS_SETUP\"; [[ -f \"$WS_SETUP\" ]] && source \"$WS_SETUP\"; until ros2 action list 2>/dev/null | grep -q \"${action}\"; do sleep 0.5; done"
}

wait_for_service() {
  local srv="$1"
  local timeout="${2:-30}"
  echo "==> wait service $srv (${timeout}s)"
  timeout "$timeout" bash -c "source \"$ROS_SETUP\"; [[ -f \"$WS_SETUP\" ]] && source \"$WS_SETUP\"; until ros2 service list 2>/dev/null | grep -q \"${srv}\"; do sleep 0.5; done"
}

wait_for_arm_controller() {
  local timeout="${1:-60}"
  echo "==> wait panda_arm_controller active (${timeout}s)"
  if ! timeout "$timeout" bash -c "source \"$ROS_SETUP\"; [[ -f \"$WS_SETUP\" ]] && source \"$WS_SETUP\";
    until ros2 control list_controllers 2>/dev/null | grep -q 'panda_arm_controller.*active'; do sleep 0.5; done"; then
    echo "ERROR: panda_arm_controller not active — check $LOG_DIR/gazebo_panda.log for URDF/Gazebo spawn errors" >&2
    return 1
  fi
}
