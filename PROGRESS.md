# PROGRESS.md — 课设进度追踪

> 与 [DESIGN.md](./DESIGN.md)（架构契约）和 [CLAUDE.md](./CLAUDE.md)（运维命令）配合使用。  
> **最后更新**：2026-06-02 · **整体完成度（粗估）**：~65%

状态图例：`✅ 已验证` · `🟡 代码有/部分验证` · `❌ 未做` · `⚠️ 阻塞/已知问题`

---

## 1. 总览

| 维度 | 状态 | 说明 |
|------|------|------|
| 架构与接口（DESIGN） | 🟡 | `DESIGN.md` draft；MCP schema / 错误码 / blackboard 已在代码中对齐 |
| 构建与 Docker | 🟡 | 镜像 `robot-homework-ros:humble`；`bootstrap` 常需手动补 apt（libfranka 等） |
| Phase 0 仿真底座 | ✅ | 桌面世界 + 俯视 RGB-D + 桥接 + TF（容器内 `verify_phase0` 通过） |
| Phase 1 感知 + blackboard | ✅ | HSV+深度+`/scene_state`（`verify_perception` 通过；多色块仍不稳） |
| Phase 2 内环 pick/place | 🟡 | 节点与 action 齐全；RViz 路径可起栈；真抓取需 Gazebo 臂 + 夹爪 |
| MCP 中间层 | 🟡 | `mcp_pick_place_brain.py` 完整；未接 MCP 客户端端到端 |
| LLM / 语音外环 | 🟡 | prompt + `resolve_user_input.py`；Whisper / ReAct 闭环未验 |
| Git | ⚠️ | 大量实现仍为未提交工作区文件 |

---

## 2. 分阶段进度（对照 DESIGN）

### Phase 0 — Gazebo 桌面 + 相机（§7）

| 项 | 状态 | 备注 |
|----|------|------|
| `pick_place_desk.sdf` 桌面与色块 | ✅ | 渲染引擎用 `ogre`（Docker 无 GPU 时避免全黑画面） |
| 俯视 RGB-D 模型 + `ros_gz_bridge` | ✅ | `/camera/color\|depth/image_raw` ~30–50 Hz |
| `overhead_camera_static.urdf` TF | ✅ | `world → camera_optical_frame` |
| `gazebo_desk_only.launch.py` | ✅ | 有 `DISPLAY` 开 GUI；无则 `-s` 无头 |
| Franka 臂在 Gazebo 中运动 | ❌ | 见 Phase 2 Gazebo / 世界合并 |

**一键启动**：`./scripts/run_in_container.sh start-phase01`（仅 Phase 0 部分）  
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
| `moveit_servo.launch.py` | 🟡 | 依赖 `my_panda_moveit_config` |
| `allow_gripper_skip`（RViz 联调） | ✅ | `pick_place_params_rviz.yaml` |
| `pick_object` / `place_at` action | 🟡 | smoke 脚本有；RViz 下无真夹爪 |
| 滑移检测 / 看门狗 / abort | 🟡 | 代码有；全栈未回归 |
| 感知物体写入 planning scene | ❌ | DESIGN §5.4 未实现 |
| `MOTION_COLLISION` 独立上报 | ❌ | 仍多映射为 `MOTION_PLANNING_FAILED` |

**路径 A — RViz（推荐联调）**

```bash
./scripts/run_in_container.sh start-phase2-rviz
./scripts/run_in_container.sh verify-phase2
./scripts/run_in_container.sh smoke   # 可选，耗时长
```

**路径 B — Gazebo + Franka（实验）**

```bash
./scripts/run_in_container.sh start-phase2-gazebo   # 世界合并未完成
```

---

### 外环 — MCP / LLM / 语音（§3–§4）

| 项 | 状态 | 备注 |
|----|------|------|
| `mcp_pick_place_brain.py` 四工具 + 错误 JSON | 🟡 | 需 ROS 栈在线；MCP 客户端未验 |
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
| `panda_pick_place` 编译失败（`$libexec`） | `setup.cfg` |
| `bootstrap_workspace.sh` 因 `set -u` 失败 | 改为 `set -eo` |
| 验证脚本管道与 `check()` 优先级 | `verify_phase0.sh` |
| 无头模式覆盖宿主机 X11 | launch 按 `DISPLAY` 决定是否 `-s` |

---

## 5. 已知缺口与风险（§9）

| 优先级 | 项 | 影响 |
|--------|-----|------|
| P0 | `pick_place_desk.sdf` 与 Franka 默认 Gazebo 世界 **未合并** | 无法在仿真里对真臂做桌面 pick-place |
| P0 | 全栈 `smoke` / 真 pick-place **未稳定跑通** | 课设演示主路径未完成 |
| P1 | 容器内 **NVIDIA 未透传** | 长期应用 GPU 渲染需修 compose |
| P1 | HSV 只稳定检出部分色块 | LLM 任务可能找不到 `red_block_01` |
| P2 | `bootstrap` / `rosdep` 不全自动 | 新环境需手动 `apt install ros-humble-libfranka` 等 |
| P2 | Panda 命名 vs `robot_type:=fer` 未统一 | MoveIt 与 Franka Gazebo 对齐待做 |
| P3 | 改动大量 **未 git commit** | 协作与回滚风险 |

---

## 6. 验证记录（可追加）

| 日期 | 环境 | 命令 / 结果 |
|------|------|-------------|
| 2026-06-02 | Docker `ros2_gazebo_dev`，`DISPLAY=:1` | `verify_phase0`：4/4 PASS |
| 2026-06-02 | 同上 | `verify_perception`：PASS（`blue_plate_01`） |
| 2026-06-02 | 同上 | `start_phase01.sh`：一键起栈 OK |
| 2026-06-02 | 同上 | `start_phase2_rviz` / 全栈 smoke：**待记录** |
| 2026-06-02 | 同上 | Gazebo 空场景修复：SDF1.8+内联相机+server/GUI 分离启动；`stop_stack` 杀光 ign |

**Gazebo 看不见物体？** 先 `./scripts/run_in_container.sh stop`，再 `start-gazebo-desk`；关掉**所有**旧 Gazebo 窗口，等**新** GUI 弹出；在 3D 视图按 `r` 重置视角。

---

## 7. 建议下一步（按顺序）

1. [ ] 合并 Franka Gazebo 世界与 `pick_place_desk.sdf`（消掉 `gazebo_pick_place.launch.py` TODO）
2. [ ] 在 Gazebo+臂路径跑通 `smoke_pick_place.sh`（真夹爪，关掉 `allow_gripper_skip`）
3. [ ] 调 HSV / 材质，使红/绿/蓝块均进 `/scene_state`
4. [ ] 配置 MCP 客户端，实测 `scan_scene → pick → place` 与 LLM prompt
5. [ ] 将当前工作区 **提交 git**（至少 `ros2_ws` 课设包 + `scripts` + `PROGRESS.md`）

---

## 8. 如何更新本文档

完成一项后请更新：

1. 对应表格中的 **状态** 与 **备注**  
2. **§6 验证记录** 增加一行（日期、环境、命令、PASS/FAIL）  
3. **§1 总览** 粗估完成度  
4. **§7** 勾选已完成项  

架构或接口变更时只改 [DESIGN.md](./DESIGN.md)；命令变更时改 [CLAUDE.md](./CLAUDE.md)。
