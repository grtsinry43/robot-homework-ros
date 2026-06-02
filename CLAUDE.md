# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> 架构决策与接口契约见 [DESIGN.md](./DESIGN.md)。课设进度见 [PROGRESS.md](./PROGRESS.md)。本文档只覆盖运维与命令层面的纪律。

## Repository layout

This repo is a ROS 2 Humble workspace for controlling a Franka Panda arm through MCP (Model Context Protocol) tool calls — i.e. an LLM client speaks MCP, the bridge converts each tool call into ROS 2 messages / MoveIt2 actions.

- `Dockerfile` / `docker-compose.yml` — dev environment (Humble + Gazebo/gz deps). Builds image `robot-homework-ros:humble`.
- `repos/vendor.repos` + `scripts/setup_vendor.sh` — pulls official `franka_ros2` (Humble) into `ros2_ws/src/vendor/` (gitignored).
- `scripts/bootstrap_workspace.sh` — rosdep + colcon build (run inside container after vendor import).
- `ros2_ws/` — the colcon workspace.
  - `src/my_panda_moveit_config/` — MoveIt Setup Assistant config (RViz fake-controller demo).
  - `src/panda_sim_bringup/` — Phase 0 Gazebo desk world, overhead RGB-D camera, launch files.
  - `src/panda_pick_place/` — perception + pick/place executor nodes (Phase 1–2).
  - `src/pick_place_msgs/` — actions/services for inner loop.
  - `src/scene_state_msgs/` — `/scene_state` blackboard message definitions (DESIGN §5.3).
  - `src/vendor/franka_ros2/` — **not in git**; cloned by `setup_vendor.sh`.
  - `mcp_pick_place_brain.py` — **主 MCP 服务**：scan_scene / pick_object / place_at / abort + debug 工具。
  - `mcp_panda_brain.py` — 旧版，仅 execute_arm_move；将被 mcp_pick_place_brain 取代。
  - `mcp_ros_bridge.py` — legacy vehicle demo; not on the pick-and-place critical path.

MCP bridge scripts are standalone Python, not colcon packages — they import `rclpy` and `mcp.server.fastmcp` directly.

## First-time setup (after Docker is installed)

```bash
# Host: build & start container
docker compose build
docker compose up -d
docker compose exec ros2-gazebo bash

# Inside container (once)
source /opt/ros/humble/setup.bash
/root/scripts/setup_vendor.sh      # clones franka_ros2 (~large)
/root/scripts/bootstrap_workspace.sh
source /root/ros2_ws/install/setup.bash
```

Without Docker you can still edit packages under `ros2_ws/src/` on the host; build/run requires a Humble environment (Docker, VM, or native Linux).

## Full stack launch order (production)

Requires: MoveIt + moveit_servo + perception + executor + (Gazebo or RViz demo) + franka_gripper.

```bash
# Terminal 1 — MoveIt (RViz demo or Gazebo+MoveIt after Phase 0)
ros2 launch my_panda_moveit_config demo.launch.py

# Terminal 2 — moveit_servo (30 Hz Cartesian servo)
ros2 launch panda_pick_place moveit_servo.launch.py

# Terminal 3 — Gazebo desk + camera (when simulation ready)
ros2 launch panda_sim_bringup gazebo_desk_only.launch.py
# OR full Franka Gazebo:
# ros2 launch panda_sim_bringup gazebo_pick_place.launch.py robot_type:=fer load_gripper:=true

# Terminal 4 — perception + pick/place executor
ros2 launch panda_pick_place pick_place.launch.py

# Terminal 5 — MCP server
python3 /root/ros2_ws/mcp_pick_place_brain.py
```

Inner loop (`executor_node`): MoveIt 宏观接近 → moveit_servo P 控制收敛 → franka_gripper grasp/move → 抬升 → 滑移验证。

Perception (`perception_node`): HSV + 深度反投影 + TF → `/scene_state`（无 fake/simulate 模式）。

MCP tools return JSON strings per DESIGN.md §4.2–§4.3.

## LLM prompt & MCP client

- System prompt template: `prompts/llm_system_prompt.md` (ReAct + error recovery table)
- MCP client example: `config/mcp_client.example.json` (replace absolute path)

## Speech input (optional)

```bash
python3 scripts/resolve_user_input.py --mode text --text "把红方块放到蓝盘子里"
# With faster-whisper installed:
python3 scripts/resolve_user_input.py --mode speech --audio recording.wav --json
```

## Stack start scripts (recommended)

On the host (X11 + Docker):

```bash
./scripts/run_in_container.sh start-phase01      # Gazebo desk + perception + executor
./scripts/run_in_container.sh verify-phase0
./scripts/run_in_container.sh verify-phase1
./scripts/run_in_container.sh start-phase2-rviz  # + MoveIt demo + moveit_servo (RViz GUI)
./scripts/run_in_container.sh verify-phase2
./scripts/run_in_container.sh smoke              # pick/place actions (long timeout)
./scripts/run_in_container.sh stop
```

Inside the container, the same scripts live under `/root/scripts/` (`start_phase01.sh`, `start_phase2_rviz.sh`, `stop_stack.sh`).

## Smoke test (requires full stack running)

```bash
./scripts/smoke_pick_place.sh
```

## Common commands

All run commands assume you are inside the dev container and have sourced `install/setup.bash`.

```bash
# Rebuild project packages only
colcon build --symlink-install --packages-select scene_state_msgs pick_place_msgs panda_pick_place panda_sim_bringup my_panda_moveit_config

# MoveIt RViz demo (fake controllers — no Gazebo)
ros2 launch my_panda_moveit_config demo.launch.py

# Phase 0: desk + camera only (no Franka vendor deps required for world half)
ros2 launch panda_sim_bringup gazebo_desk_only.launch.py

# Phase 0: full stack (requires franka_gazebo_bringup built)
ros2 launch panda_sim_bringup gazebo_pick_place.launch.py robot_type:=fer load_gripper:=true

# MCP — main pick-and-place server
python3 /root/ros2_ws/mcp_pick_place_brain.py
```

## Phase 0 verification checklist (when Docker/Gazebo available)

1. `ros2 topic list | grep camera` — expect `/camera/color/image_raw`, `/camera/depth/image_raw`
2. `ros2 run tf2_ros tf2_echo world camera_optical_frame` — static TF from `overhead_camera_static.urdf`
3. Franka joint states publishing after `gazebo_pick_place.launch.py`
4. Known gap: merging `pick_place_desk.sdf` with franka default world — see TODO in `gazebo_pick_place.launch.py`

## Architecture notes that matter

**MCP ↔ ROS 2 threading.** Both bridge scripts run `rclpy.spin()` on a daemon thread and let `mcp.run()` own the main thread. Do not block the main thread with ROS calls (e.g. `wait_for_server`) — `mcp_panda_brain.py` documents this with "致命修复 3" and instead checks `server_is_ready()` per call and bails out if MoveIt2 isn't up. Preserve that pattern when adding new tools.

**stdout is sacred.** MCP communicates with the LLM client over stdio. `mcp_panda_brain.py` sets `RCUTILS_LOGGING_USE_STDERR=1` *before* importing `rclpy` to keep ROS 2's C++ logs off stdout. Any new MCP server in this repo must do the same.

**MoveIt2 action name.** The action server is exposed at `move_action`, not `/move_group/move_action`.

**Franka naming.** Official `franka_ros2` uses `robot_type:=fer` (Franka Emika Robot). Legacy `moveit_resources_panda` / `my_panda_moveit_config` still say "panda" — reconcile when wiring MoveIt to Gazebo.

**MoveIt config is generated.** Prefer regenerating via Setup Assistant over hand-editing SRDF/kinematics.

There is no test suite, lint config, or CI in this repo.
