# robot-homework-ros

语音/文字驱动的 Franka 桌面 pick-and-place 课设（ROS 2 Humble + Gazebo + MCP + LLM 外环）。

## 文档（建议阅读顺序）

| 文件 | 内容 |
|------|------|
| [PROGRESS.md](./PROGRESS.md) | **进度与待办**（接手先看） |
| [DESIGN.md](./DESIGN.md) | 架构、MCP 契约、错误码 |
| [CLAUDE.md](./CLAUDE.md) | Docker、build、启动命令 |
| [prompts/llm_system_prompt.md](./prompts/llm_system_prompt.md) | LLM ReAct system prompt |
| [config/mcp_client.example.json](./config/mcp_client.example.json) | MCP 客户端示例 |

## Agent skills

- Cursor / Codex：`.agents/skills/`（协作规范见 `grt-collaborating`）
- Claude Code：`.claude/skills` → 指向 `.agents/skills`

## 快速启动（Docker + 宿主机 X11）

```bash
xhost +local:docker
docker compose build && docker compose up -d

./scripts/run_in_container.sh start-gazebo-desk   # 仅 Gazebo 桌面（无机械臂）
./scripts/run_in_container.sh start-phase01       # 桌面 + 感知
PHASE01_USE_GUI=false ./scripts/run_in_container.sh start-phase01  # WSLg GUI 崩溃时
./scripts/run_in_container.sh start-phase2-rviz   # + MoveIt RViz（仍无 Gazebo 臂）
./scripts/run_in_container.sh stop
```

容器内首次构建：

```bash
docker compose exec ros2-gazebo bash
/root/scripts/setup_vendor.sh
/root/scripts/bootstrap_workspace.sh
source /root/ros2_ws/install/setup.bash
```

Docker 镜像会根据 `requirements.txt` 安装 MCP / demo scripts 需要的 Python 依赖。

## 快速启动（WSL/本机 ROS 2）

在 Ubuntu 22.04 + ROS 2 Humble 环境中：

```bash
cd ~/robot-homework-ros
./scripts/setup_vendor.sh
./scripts/bootstrap_workspace.sh
python3 -m pip install -r requirements.txt   # MCP / demo scripts

# WSLg: keep DISPLAY for Gazebo RGB-D rendering, but skip the Gazebo GUI client.
DISPLAY=:0 PHASE01_USE_GUI=false bash scripts/start_phase01.sh
bash scripts/verify_phase0.sh
bash scripts/verify_perception.sh
bash scripts/stop_stack.sh
```

`PHASE01_USE_GUI=auto|true|false` controls only the Gazebo GUI client. RGB-D sensors still need a working `DISPLAY`; on WSLg, `DISPLAY=:0 PHASE01_USE_GUI=false` avoids the observed Gazebo GUI / OGRE crash while keeping camera rendering alive.

## 当前范围说明

- **已有**：桌面仿真、相机、HSV 感知、`/scene_state`、内环/MCP 代码、启动与验证脚本  
- **未有**：Gazebo 中 Franka 与桌面世界合并、端到端真抓放、LLM/MCP 联调闭环  

vendor：`ros2_ws/src/vendor/franka_ros2` 由 `scripts/setup_vendor.sh` 拉取，不纳入 git。
