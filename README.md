# robot-homework-ros

语音/文字驱动的 Franka 桌面 pick-and-place 课设（ROS 2 Humble + Gazebo + MCP + LLM 外环）。

## 文档（建议阅读顺序）

| 文件 | 内容 |
|------|------|
| README.md | **当前交接快照**（接手先看这一页） |
| [PROGRESS.md](./PROGRESS.md) | 详细历史进度追踪；可能滞后于 README 的最新验证记录 |
| [DESIGN.md](./DESIGN.md) | 架构、MCP 契约、错误码 |
| [CLAUDE.md](./CLAUDE.md) | Docker、build、启动命令 |
| [prompts/llm_system_prompt.md](./prompts/llm_system_prompt.md) | LLM ReAct system prompt |
| [ros2_ws/src/panda_pick_place/config/atomic_actions.json](./ros2_ws/src/panda_pick_place/config/atomic_actions.json) | 协作扩展用 atomic action library |
| [config/mcp_client.example.json](./config/mcp_client.example.json) | MCP 客户端示例 |

## 当前交接快照（2026-06-10）

当前分支已从「单步 MCP 工具 + 感知 demo」推进到「ROS-LLM 风格能力目录 / observation manager / 短线性 plan executor + Phase 2 logical pick-place smoke」：

- **已实测通过（WSL 本机 ROS 2 Humble）**
  - `colcon build --symlink-install --packages-select panda_pick_place`
  - `python3 scripts/validate_plan_executor.py`
  - `GAZEBO_USE_GUI=0 bash scripts/start_phase2_unified.sh`
  - `bash scripts/verify_phase2.sh`
  - `bash scripts/smoke_pick_place.sh`：`pick red_block_01` → `place above blue_plate_01` 成功
- **当前 smoke 的性质**
  - Phase 2 是单 Gazebo 场景：桌面、色块、相机、Panda、MoveIt 内环在同一栈里。
  - 默认配置仍是 demo/logical grasp：`allow_gripper_skip=true`、`skip_all_servo=true`。
  - 这证明 action、MoveIt 轨迹、执行器状态机、语义 place、脚本判定链路已通；还不等于真实夹爪物理抓取已完成。
- **已接入 ROS-LLM 风格能力**
  - `get_action_library()`：显式 atomic action library，协作者加能力先改这里。
  - `get_robot_context()`：observation manager，返回 scene freshness、readiness、workspace、last_error、recent_events、recommended_next_step。
  - `execute_plan(plan_json)`：短线性结构化计划校验和执行；拒绝 debug/raw motion/递归/未知参数/非法 enum。
- **最重要的未完成项**
  - 真夹爪路径：`allow_gripper_skip=false`、`/franka_gripper/*` action、真实夹取/释放、物体 attachment/collision 还没有闭环验证。
  - MCP 客户端端到端：plan executor 已本地校验，但还没通过 Cursor/Claude MCP client 跑完整自然语言任务。
  - 感知主路径：当前是 HSV + Gazebo sim layout fallback；YOLO 尚未接入。
  - Docker：本轮最终验证在 WSL 本机完成；Docker 在当前 WSL 环境不可用，合并后建议有 Docker 的机器补跑一次。

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
./scripts/run_in_container.sh start-phase2-unified # 桌面 + Panda + MoveIt 内环（单 Gazebo 场景）
./scripts/run_in_container.sh verify-phase2
./scripts/run_in_container.sh smoke
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

Phase 2 logical smoke（本轮验证路径）：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# 保留 DISPLAY 给 Gazebo RGB-D sensor 渲染，但禁用 Gazebo GUI client，避免 WSLg OGRE 崩溃。
DISPLAY=:0 GAZEBO_USE_GUI=0 bash scripts/start_phase2_unified.sh

bash scripts/verify_phase2.sh
bash scripts/smoke_pick_place.sh
bash scripts/stop_stack.sh
```

不要把 `DISPLAY` 清空来跑 Phase 2：Gazebo server 里的 RGB-D camera 渲染也需要它。只禁用 GUI client，用 `GAZEBO_USE_GUI=0`。

## 当前范围说明

- **已有并验证**：桌面仿真、相机、HSV/sim fallback 感知、`/scene_state`、单 Gazebo 场景 Panda + MoveIt、pick/place ROS action、logical smoke、atomic action library、robot context、短线性 plan executor、本机 WSL 启动与验证脚本。
- **已有但待更深验证**：MCP server 工具集、`execute_plan` 接入 MCP bridge、recent event feedback、abort/cancel 日志、MoveIt servo 兜底路径。
- **未完成**：真夹爪物理抓放、物体 attach/detach 到 planning scene、YOLO 主感知、MCP 客户端自然语言端到端、Docker 环境下最新 Phase 2 回归。

## 代码审计摘记

- `executor_node.py`
  - 提供 `/pick_place/pick_object`、`/pick_place/place_at`、`/pick_place/abort`。
  - 已补 goal/execute/result/cleanup 日志，方便定位 action 卡住、任务未释放、held 状态异常。
  - `allow_gripper_skip=true` 时使用安全模拟抓取高度，避免 Panda finger 与 link5 自碰撞。
  - place approach 拆成 clearance waypoint，再移动到释放点，已在 smoke 中看到 `place_clearance_translate` feedback。
- `moveit_helper.py`
  - 不再在 action callback 内 `spin_until_future_complete(self._node, ...)`，改为轻量轮询等待 future，避免 executor 回调卡死。
  - MoveIt 成功后短暂 settle，降低连续 goal 时 current state 尚未追上导致 `Invalid Trajectory` 的概率。
- `scripts/smoke_pick_place.sh`
  - 现在解析 action result 的 `success: true`，`success: false` 会失败；不会再出现 place aborted 但脚本 `[PASS]` 的假阳性。
- `scripts/start_phase2_rviz.sh`
  - `phase2_rviz.launch.py` 已 include `moveit_servo.launch.py`；启动脚本不再单独再起一个 `/servo_node`。
  - 新增 `GAZEBO_USE_GUI=0`，用于 WSLg 禁 GUI client 但保留 RGB-D 渲染。
- `scripts/lib/stack_common.sh`
  - 加强 `ign gazebo`、`gripper_joint_state_merger` 清理。旧 Gazebo 残留会污染 `/clock`、`/stats`，甚至让 controller_manager 起不来。

## 协作扩展：Atomic Action Library

机器人可被 LLM/MCP 调用的正式能力登记在
[`ros2_ws/src/panda_pick_place/config/atomic_actions.json`](./ros2_ws/src/panda_pick_place/config/atomic_actions.json)。
它是协作者添加新功能时的接口契约：先登记 action 的名字、入口、输入输出、前置条件、错误码和安全边界，再实现 ROS action/service 或 MCP tool。

校验契约：

```bash
python3 scripts/validate_action_library.py
```

当前 MCP server 也暴露 `get_action_library()`，客户端可以在任务开始前查询机器人能力目录。

## 运行时观察：Robot Context

MCP server 暴露 `get_robot_context()` 作为 ROS-LLM 风格的 observation manager。它不触发扫描、不移动机械臂，只汇总当前运行时状态：

- `/scene_state` 是否存在、是否新鲜、当前可见 object ids
- perception service、pick/place action、abort service、MoveIt、gripper readiness
- `panda_link0` 工作空间边界
- MCP 最近一次失败错误与建议下一步
- 最近 MCP 工具事件（`recent_events`），作为后续 task feedback / retry 策略的短期历史

协作者新增底层能力时，除了更新 atomic action library，也应考虑该能力是否需要纳入 `get_robot_context()` 的 readiness 或 observation 字段。

## 结构化计划：Plan Executor

MCP server 暴露 `execute_plan(plan_json)`，把项目从「LLM 单步调工具」推进到「LLM 输出结构化计划，系统校验并执行」的第一版。

`plan_json` 是 JSON 字符串，内容是短线性步骤数组：

```json
[
  {"tool": "pick_object", "args": {"id": "red_block_01"}},
  {"tool": "place_at", "args": {"target_id": "blue_plate_01", "offset": "above"}}
]
```

执行前会校验：

- step tool 必须登记在 atomic action library
- 只允许 `get_robot_context`、`scan_scene`、`pick_object`、`place_at`、`abort_current_task`
- 禁止 debug 工具、递归 `execute_plan`、未知参数、缺少必填参数、非法 enum
- 最多 8 步；第一版不支持条件、循环、变量绑定、raw coordinate motion

因此当用户只给自然语言描述时，LLM 仍应先 `scan_scene` 确认 object id，再把已具体化的步骤交给 `execute_plan`。执行中任一步 `failed` 或 `aborted`，plan 会立即停止，并返回每步结果、失败点和 `final_context`，供下一轮反馈恢复使用。

轻量校验：

```bash
python3 scripts/validate_plan_executor.py
```

vendor：`ros2_ws/src/vendor/franka_ros2` 由 `scripts/setup_vendor.sh` 拉取，不纳入 git。
