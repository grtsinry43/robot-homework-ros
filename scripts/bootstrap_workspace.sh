#!/usr/bin/env bash
# One-shot workspace bootstrap (run inside ROS 2 Humble environment, e.g. Docker).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble not sourced. Run: source /opt/ros/humble/setup.bash"
  exit 1
fi

source /opt/ros/humble/setup.bash

if [[ ! -d "${WS}/src/vendor/franka_ros2" ]]; then
  echo "==> Vendor deps missing; running setup_vendor.sh"
  "${ROOT}/scripts/setup_vendor.sh"
fi

cd "${WS}"

echo "==> rosdep install"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "==> colcon build (project packages first)"
colcon build --symlink-install \
  --packages-select scene_state_msgs pick_place_msgs panda_pick_place panda_sim_bringup

echo "==> colcon build (Franka Gazebo stack — may take several minutes)"
colcon build --symlink-install \
  --packages-up-to franka_gazebo_bringup franka_example_controllers

echo "==> colcon build (MoveIt config)"
colcon build --symlink-install \
  --packages-select my_panda_moveit_config

echo "==> Bootstrap complete. Source:"
echo "    source ${WS}/install/setup.bash"
