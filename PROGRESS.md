# PROGRESS.md — 课设进度追踪

> 与 [DESIGN.md](./DESIGN.md)（架构契约）和 [CLAUDE.md](./CLAUDE.md)（运维命令）配合使用。  
> **最后更新**：2026-06-08 · **整体完成度（粗估）**：~72%

状态图例：`✅ 已验证` · `🟡 代码有/部分验证` · `❌ 未做` · `⚠️ 阻塞/已知问题`

---

## 1. 总览

| 维度 | 状态 | 说明 |
|------|------|------|
| 架构与接口（DESIGN） | 🟡 | `DESIGN.md` draft；MCP schema / 错误码 / blackboard 已在代码中对齐 |
| 构建与 Docker | 🟡 | 镜像 `robot-homework-ros:humble`；`bootstrap` 常需手动补 apt（libfranka 等） |
| Phase 0 仿真底座 | ✅ | 桌面世界 + 俯视 RGB-D + 桥接 + TF（容器内 `verify_phase0` 通过） |
| Phase 1 感知 + blackboard | ✅ | 桌面位姿已标定（`verify_perception` 桌面 envelope PASS） |
| Phase 2 内环 pick/place | 🟡 | pick 已 SUCCEEDED；place 待稳；默认 `skip_all_servo`（避免 RViz 假臂循环） |
| MCP 中间层 | 🟡 | `mcp_pick_place_brain.py` 完整；未接 MCP 客户端端到端 |
| LLM / 语音外环 | 🟡 | prompt + `resolve_user_input.py`；Whisper / ReAct 闭环未验 |
| Git | ✅ | 当前基线已提交；后续协作改动按分支/PR 记录 |

---

## 2. 分阶段进度（对照 DESIGN）

### Phase 0 — Gazebo 桌面 + 相机（§7）

| 项 | 状态 | 备注 |
|----|------|------|
| `pick_place_desk.sdf` 桌面与色块 | ✅ | 渲染引擎用 `ogre`（Docker 无 GPU 时避免全黑画面） |
| 俯视 RGB-D 模型 + `ros_gz_bridge` | ✅ | `/camera/color\|depth/image_raw` ~30–50 Hz |
| `overhead_camera_static.urdf` TF | ✅ | `world → camera_optical_frame` |
| `gazebo_desk_only.launch.py` | ✅ | 有 `DISPLAY` 可开 GUI；WSLg 建议 `PHASE01_USE_GUI=false` 保留相机渲染但跳过 GUI client |
| Franka 臂在 Gazebo 中运动 | 🟡 | `gazebo_pick_place.launch.py` 已传 `pick_place_desk.sdf`；全栈待验 |

**一键启动**：`./scripts/run_in_container.sh start-phase01`（仅 Phase 0 部分；本机 WSL 可用 `DISPLAY=:0 PHASE01_USE_GUI=false bash scripts/start_phase01.sh`）  
**验证**：`./scripts/run_in_container.sh verify-phase0`

---

### Phase 1 — 感知 + `/scene_state`（§5.2–§5.3）

| 项 | 状态 | 备注 |
|----|------|------|
| `scene_state_msgs` / `pick_place_msgs` | ✅ | 已 colcon |
| `perception_node`（HSV + 深度 + TF） | ✅ | 已修 `tf2_geometry_msgs` |
| YOLO 主路径 | ❌ | 设计允许；当前 **HSV 兜底**，报告需说明 |
| `object_id` 命名 `<label>_NN` | ✅ | 例：`blue_plate_01` |
| 多物体稳定检测（红/绿/蓝） | ⚠️ | 常只稳检 1 个；HSV 已略放宽，待调 |
| desk-only 下 `world→panda_link0` | 🟡 | **静态 TF** 桥接；非臂的真实 TF |

**验证**：`./scripts/run_in_container.sh verify-phase1`

---

### Phase 2 — 内环执行（§5.1、§5.4）

| 项 | 状态 | 备注 |
|----|------|------|
| `executor_node`（MoveIt 接近 + servo + 夹爪） | 🟡 | 可启动；Humble 已去掉 `ServoCommandType` |
| `moveit_servo.launch.py` | 🟡 | 依赖 `my_panda_moveit_config`（`.setup_assistant` 空文件已修） |
| `allow_gripper_skip`（RViz 联调） | ✅ | `pick_place_params_rviz.yaml` |
| `pick_object` / `place_at` action | 🟡 | smoke 可发 goal；RViz 无夹爪；位姿错时 `OUT_OF_REACH` |
| 滑移检测 / 看门狗 / abort | 🟡 | 代码有；全栈未回归 |
| 感知物体写入 planning scene | ❌ | DESIGN §5.4 未实现 |
| `MOTION_COLLISION` 独立上报 | ❌ | 仍多映射为 `MOTION_PLANNING_FAILED` |

**路径 A — Gazebo 单窗口（推荐，原 start-phase2-rviz 已改）**

```bash
./scripts/run_in_container.sh stop          # 先停掉旧栈，关掉所有 RViz/Gazebo 窗口
./scripts/run_in_container.sh start-phase2-unified   # 或 start-phase2-rviz（等价）
# 只看 Gazebo：桌面 + 色块 + Franka；默认不启 RViz（避免假臂循环抖动）
# 调试才开 RViz: LAUNCH_RVIZ=1（与 Gazebo 仍非同一物理仿真）
./scripts/run_in_container.sh verify-phase2
./scripts/run_in_container.sh smoke
```

**路径 B — Gazebo + Franka（实验）**

```bash
./scripts/run_in_container.sh start-phase2-gazebo   # 桌面世界经 gz_args 合并；MoveIt↔Gazebo 待验
```

---

### 外环 — MCP / LLM / 语音（§3–§4）

| 项 | 状态 | 备注 |
|----|------|------|
| `mcp_pick_place_brain.py` 四工具 + 错误 JSON | 🟡 | 需 ROS 栈在线；`requirements.txt` 已显式列出 Python `mcp` 依赖；MCP 客户端未验 |
| `prompts/llm_system_prompt.md` | ✅ | |
| `config/mcp_client.example.json` | ✅ | |
| `scripts/resolve_user_input.py`（Whisper） | 🟡 | 可选依赖未在课设硬件验证 |
| 前端 / 语音开关 UI | ❌ | 仅 CLI |

---

## 3. 启动与验证脚本

| 脚本 | 用途 |
|------|------|
| [`scripts/run_in_container.sh`](./scripts/run_in_container.sh) | **宿主机入口**（xhost + docker compose） |
| [`scripts/start_phase01.sh`](./scripts/start_phase01.sh) | Phase 0+1 后台栈 |
| [`scripts/start_phase2_rviz.sh`](./scripts/start_phase2_rviz.sh) | Phase 2 RViz 全栈 |
| [`scripts/start_phase2_gazebo.sh`](./scripts/start_phase2_gazebo.sh) | Phase 2 Gazebo（实验） |
| [`scripts/stop_stack.sh`](./scripts/stop_stack.sh) | 停止后台进程 |
| [`scripts/verify_phase0.sh`](./scripts/verify_phase0.sh) | 相机话题 / Hz / TF |
| [`scripts/verify_perception.sh`](./scripts/verify_perception.sh) | trigger_scan + scene_state |
| [`scripts/verify_phase2.sh`](./scripts/verify_phase2.sh) | move_action + actions + servo |
| [`scripts/smoke_pick_place.sh`](./scripts/smoke_pick_place.sh) | CLI pick/place（自动取 scene id） |
| [`scripts/setup_vendor.sh`](./scripts/setup_vendor.sh) | 拉取 `franka_ros2` |
| [`scripts/bootstrap_workspace.sh`](./scripts/bootstrap_workspace.sh) | rosdep + colcon（常需补 apt） |

日志与 PID：`/tmp/robot_homework_ros/logs/`、`/tmp/robot_homework_ros/*.pid`（容器内）

---

## 4. 测试中发现并已修复的问题

| 问题 | 修复位置 |
|------|----------|
| Launch 调用不存在的 `gz_sim` 可执行文件 | `gazebo_desk_only.launch.py` → `ros_gz_sim` 的 `gz_sim.launch.py` |
| Docker 下相机画面全黑（ogre2/EGL） | `pick_place_desk.sdf`：`ogre2` → `ogre` |
| `perception_node` TF 崩溃 | `import tf2_geometry_msgs` |
| `executor_node` 无法 import `ServoCommandType` | `servo_helper.py` 适配 Humble |
| MoveIt `demo.launch.py`：`NoneType.get` | `config/moveit_setup_assistant.yaml` + CMake install |
| `smoke_pick_place.sh` 解析不到 `object_id` | `sed` 提取 `id:` 字段 |
| 感知位姿错乱（`OUT_OF_REACH`） | Gazebo 深度=range + 修正 fx + `camera_static_tf.launch.py` |
| Gazebo `camera_info` fx 与 SDF 不符 | `override_camera_intrinsics`（fx≈554） |
| 僵尸 `overhead_camera_tf` 污染 TF | `stop_stack` 加强清理；改用静态 TF 发布相机 |
| MoveIt demo `no ros2_control tag` | URDF 改用 `panda.urdf.xacro` + `moveit_controllers.yaml` |
| action 回调内 `spin_once` 死锁 | `MultiThreadedExecutor` + `spin_until_future_complete` |
| `panda_pick_place` 编译失败（`$libexec`） | `setup.cfg` |
| `bootstrap_workspace.sh` 因 `set -u` 失败 | 改为 `set -eo` |
| 验证脚本管道与 `check()` 优先级 | `verify_phase0.sh` |
| 无头模式覆盖宿主机 X11 | launch 按 `DISPLAY` 决定是否 `-s` |
| WSLg Gazebo GUI client 崩溃（OGRE/GL3PlusTextureGpu） | `start_phase01.sh` 新增 `PHASE01_USE_GUI=auto|true|false`；WSL 自动禁用 GUI client |

---

## 5. 已知缺口与风险（§9）

| 优先级 | 项 | 影响 |
|--------|-----|------|
| P1 | smoke 全绿 | 伺服丢目标时用 fallback；优先抓方块而非蓝盘 |
| P0 | Gazebo 臂 + MoveIt 控制器 **未对齐** | `start-phase2-gazebo` 待验 |
| P1 | 多色块 HSV 仍不稳 | 常只稳检蓝盘；红/绿待调 |
| P1 | 容器内 **NVIDIA 未透传** | 长期应用 GPU 渲染需修 compose |
| P1 | HSV 只稳定检出部分色块 | LLM 任务可能找不到 `red_block_01` |
| P1 | WSLg Gazebo GUI 不稳定 | Phase 0/1 用 `PHASE01_USE_GUI=false`；只保留 `DISPLAY` 给 RGB-D sensor |
| P2 | `bootstrap` / `rosdep` 不全自动 | 新环境需手动 `apt install ros-humble-libfranka` 等 |
| P2 | 空 `.setup_assistant` 导致 MoveIt demo 崩溃 | 已用 `config/moveit_setup_assistant.yaml` 安装 |
| P2 | Panda 命名 vs `robot_type:=fer` 未统一 | MoveIt 与 Franka Gazebo 对齐待做 |
| P3 | 协作分支 / PR / 验证流程未固化 | 多人开发需约定分支命名、验证记录、push 权限 |

---

## 6. 验证记录（可追加）

| 日期 | 环境 | 命令 / 结果 |
|------|------|-------------|
| 2026-06-02 | Docker `ros2_gazebo_dev`，`DISPLAY=:1` | `verify_phase0`：4/4 PASS |
| 2026-06-02 | 同上 | `verify_perception`：PASS（`blue_plate_01`） |
| 2026-06-02 | 同上 | `start_phase01.sh`：一键起栈 OK |
| 2026-06-02 | 同上 | `start_phase2_rviz` + `verify_phase2`：5/5 PASS |
| 2026-06-02 | 同上 | `verify_perception` 桌面位姿 PASS（蓝盘 x≈0.54 y≈-0.05 z≈0.11） |
| 2026-06-02 | 同上 | `smoke`：pick 到 `pick_servo`（MoveIt 接近成功）；曾 `CONTROL_FAILED` 已修 |
| 2026-06-02 | 同上 | Gazebo 空场景修复：SDF1.8+内联相机+server/GUI 分离启动；`stop_stack` 杀光 ign |
| 2026-06-08 | WSL 本机 ROS 2 Humble，`DISPLAY=:0`，Gazebo GUI client disabled | `verify_phase0`：4/4 PASS（color hz≈8.95）；`verify_perception`：PASS（`blue_plate_01`，桌面 envelope PASS；红/绿 HSV 待调） |

**Gazebo 看不见物体？** 先 `./scripts/run_in_container.sh stop`，再 `start-gazebo-desk`；关掉**所有**旧 Gazebo 窗口，等**新** GUI 弹出；在 3D 视图按 `r` 重置视角。

---

## 7. 建议下一步（按顺序）

1. [x] 合并 Franka Gazebo 世界与 `pick_place_desk.sdf`（`gz_args` + 正确 launch 名）
2. [ ] 在 Gazebo+臂路径跑通 `smoke_pick_place.sh`（真夹爪，关掉 `allow_gripper_skip`）
3. [ ] 调 HSV，使红/绿块也进 `/scene_state`（蓝盘位姿已 OK）
4. [ ] 配置 MCP 客户端，实测 `scan_scene → pick → place` 与 LLM prompt
5. [x] 将当前工作区 **提交 git**（感知/TF/MoveIt setup 等）

---

## 8. 如何更新本文档

完成一项后请更新：

1. 对应表格中的 **状态** 与 **备注**  
2. **§6 验证记录** 增加一行（日期、环境、命令、PASS/FAIL）  
3. **§1 总览** 粗估完成度  
4. **§7** 勾选已完成项  

架构或接口变更时只改 [DESIGN.md](./DESIGN.md)；命令变更时改 [CLAUDE.md](./CLAUDE.md)。
