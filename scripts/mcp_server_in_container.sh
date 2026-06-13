#!/usr/bin/env bash
# Bridge an MCP stdio client (e.g. Claude Code on the host) to the pick-and-place
# MCP server running INSIDE the ros2_gazebo_dev container, where ROS 2 lives.
#
# `docker exec -i` wires the host's stdin/stdout straight to the in-container python
# process, so the MCP JSON-RPC stream tunnels across the container boundary with no
# extra socket server. ROS C++ logs are forced to stderr (RCUTILS_LOGGING_USE_STDERR,
# set inside the server) so stdout stays a clean JSON-RPC channel.
#
# Register in Claude Code:  claude mcp add panda-pick-place -- /abs/path/to/this/script
set -euo pipefail

CONTAINER="${ROS_CONTAINER:-ros2_gazebo_dev}"

exec docker exec -i "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  source /root/ros2_ws/install/setup.bash >/dev/null 2>&1
  cd /root/ros2_ws
  exec python3 mcp_pick_place_brain.py
'
