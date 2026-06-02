# DESIGN.md — 机械臂语音控制课设

> Status: draft  ·  最后更新：2026-06-01  ·  本文档是架构与接口契约的单一真源。运维见 `CLAUDE.md`，进度见 `PROGRESS.md`。

---

## 1. 项目目标与范围

### 1.1 一句话目标

让用户用自然语言（语音或文字）指挥仿真环境里的 Franka Panda 机械臂完成桌面 pick-and-place 任务，由 LLM 做任务规划，由传统 ML（YOLO + 深度反投影）做感知，由 ROS 2 内环做实时视觉伺服与运动控制。

### 1.2 In scope / Out of scope

| In scope | Out of scope |
|---|---|
| Gazebo 纯仿真 | 真机（libfranka / 真 Panda） |
| 单一感知工具：YOLO + 深度反投影 | 6D 位姿估计、抓取位姿生成、点云分割 |
| 静态桌面场景（物体不动、桌面不晃） | 动态场景、运动物体追踪 |
| 中文 + 英文文字输入 + Whisper 语音输入 | 多模态视频输入、连续多轮对话记忆持久化 |
| 单 Panda 机械臂 + 一个深度相机 | 多机械臂协作、移动底盘 |
| LLM 外环规划（秒级） + ROS 内环伺服（30Hz） | 端到端 VLA 模型、强化学习策略 |

---

## 2. 整体架构

### 2.1 架构图

```
┌──────────────────────────────────────────────────────────────┐
│  用户：语音 或 文字                                            │
└────────────────────────┬─────────────────────────────────────┘
                         ↓
              ┌──────────────────────┐
              │  Whisper (本地)       │   faster-whisper, ~0.5s 延迟
              └──────────┬───────────┘
                         ↓ 文字
              ┌──────────────────────┐
              │  调度 LLM             │   外环 (秒级)
              │  ReAct 风格           │   任务分解 + 工具序列
              └──────────┬───────────┘
                         ↓ MCP tool calls
              ┌──────────────────────┐
              │  MCP Bridge           │
              │  - scan_scene         │   感知一次
              │  - pick_object        │   触发内环抓取（阻塞）
              │  - place_at           │   触发内环放置（阻塞）
              │  - abort_current_task │   全局抢占中断
              │  - set_gripper *      │   * debug 用，默认不入 prompt
              │  - execute_arm_move * │   * debug 用，默认不入 prompt
              └──────────┬───────────┘
                         ↓ ROS 2 actions / topics
   ┌─────────────────────┴────────────────────────────────────┐
   │  ROS 2 执行层                                             │
   │                                                           │
   │  ┌────────────────────┐    ┌────────────────────────┐    │
   │  │ 视觉伺服节点 30Hz   │←───│ 感知节点               │    │
   │  │  P 控制 + 收敛阈值  │    │ YOLO + 深度反投影       │    │
   │  └─────────┬──────────┘    └────────────┬───────────┘    │
   │            ↓                             ↓                │
   │  ┌────────────────────┐    ┌────────────────────────┐    │
   │  │ MoveIt2            │    │ /scene_state blackboard │    │
   │  │ 运动规划 + 控制     │    │ id → 6D pose 表         │    │
   │  └─────────┬──────────┘    └────────────────────────┘    │
   └────────────┼─────────────────────────────────────────────┘
                ↓
        ┌──────────────┐
        │  Gazebo      │   Panda + 深度相机 + 桌面物体
        └──────────────┘
```

### 2.2 时间尺度分层

机器人控制是天然的多时间尺度系统。LLM 推理延迟物理上无法进入快控制环，分层不是设计选择，是约束。

| 层 | 频率 | 谁来做 | 负责什么 |
|---|---:|---|---|
| 任务规划 | 0.1–1 Hz | LLM | "把红方块放进蓝盘子" |
| 工具调度 | 0.5–2 Hz | MCP Bridge | 工具调用与结果回传 |
| 运动规划 | 1–10 Hz | MoveIt2 | 算一条避障路径 |
| 视觉伺服 | 30 Hz | ROS 内环节点 | 看着目标微调末端位置 |
| 关节控制 | 1000 Hz | ros2_control | 关节位置/力矩指令 |

### 2.3 各层职责边界

- **LLM 决定"做什么"**，不决定"怎么做"。它不发坐标、不调夹爪、不规划路径。
- **MCP 工具是语义动作单元**。一个工具 = 一个对 LLM 完整可理解的动作（"扫描场景" / "抓起 X" / "放到 Y"）。
- **ROS 内环承担实时性、精度、安全**。失败时要主动输出语义化错误，不是默默重试到超时。

---

## 3. 外环：LLM 调度层

### 3.1 输入

- **文字**：直接提交到 LLM
- **语音**：`faster-whisper` 本地推理（small / medium 模型，CPU 也能跑）→ 输出文字 → 再走文字路径
- 文字与语音双输入并存，前端用一个开关切换

> ⚠️ 待验证：Whisper 在课设硬件上的实际延迟与显存占用，可能需要降级到 `tiny` 模型。

### 3.2 调度模型与 prompt 风格

- **模型**：Claude / GPT 任选其一，通过 MCP 协议接入。MCP 协议层与具体模型解耦，换模型不用动工具实现。
- **Prompt 范式**：ReAct（Thought → Action → Observation 循环）
- **System prompt 必含**：
  - 任务边界（仅桌面 pick-and-place，不做超出能力的事）
  - 工具清单与每个工具的语义（不重复 schema，schema 由 MCP 自动暴露）
  - 错误码到恢复策略的对照表（详见 §4.3）
  - 不可见信息约定：LLM 不会拿到坐标、不会拿到坐标系名（详见 §8 D2、D5）

### 3.3 工具调用范式

LLM 在每个用户请求后产出一个**工具调用序列**，逐个发出。每个工具同步返回结果，LLM 再决定下一步（详见 §8 D1）。

典型成功流程：

```
User: "把红方块放到蓝盘子里"
  LLM → scan_scene()
       ← {"objects": [{"id": "red_block_01", "label": "red_block"},
                      {"id": "blue_plate_01", "label": "blue_plate"}, ...]}
  LLM → pick_object(id="red_block_01")
       ← {"status": "ok"}
  LLM → place_at(target_id="blue_plate_01", offset="above")
       ← {"status": "ok"}
  LLM → "已完成，红方块放进蓝盘子。"
```

---

## 4. 中间层：MCP 工具契约

### 4.1 工具清单

LLM 默认 prompt 里**只暴露前 4 个**。后两个是 debug / 单测工具，不进 LLM 视野。

| 工具 | 阻塞 | 典型耗时 | 用途 |
|---|---|---:|---|
| `scan_scene()` | 同步 | 0.3–1s | 一次 YOLO + 深度反投影，返回场景物体 id 列表 |
| `pick_object(id)` | 同步 | 5–20s | 触发内环抓取，阻塞直到成功/失败 |
| `place_at(target_id, offset)` | 同步 | 5–15s | 触发内环放置，阻塞直到成功/失败 |
| `abort_current_task()` | 同步 | <0.5s | 抢占式中断当前阻塞工具，立即停伺服环 |
| `set_gripper(state)` * | 同步 | 1–2s | * 仅 debug：直接开/合夹爪 |
| `execute_arm_move(x,y,z)` * | 同步 | 3–10s | * 仅 debug：直接运动到坐标（已存在） |

### 4.2 输入 / 输出 Schema

#### `scan_scene()`

**输入**：无参数

**输出**：

```json
{
  "status": "ok",
  "objects": [
    {
      "id": "red_block_01",
      "label": "red_block",
      "confidence": 0.93
    },
    {
      "id": "blue_plate_01",
      "label": "blue_plate",
      "confidence": 0.88
    }
  ],
  "scanned_at": "2026-06-01T10:32:11Z"
}
```

注意：**返回值里不含坐标**。坐标写入 `/scene_state` blackboard，由后续工具内部反查（详见 §8 D2）。

#### `pick_object(id)`

**输入**：

```json
{ "id": "red_block_01" }
```

**输出（成功）**：

```json
{ "status": "ok", "id": "red_block_01" }
```

**输出（失败）**：见 §4.3 错误对象。

#### `place_at(target_id, offset)`

**输入**：

```json
{
  "target_id": "blue_plate_01",
  "offset": "above"
}
```

`offset` 枚举：`"above"` / `"left_of"` / `"right_of"` / `"front_of"` / `"behind"`。语义位置由内环节点解释为 panda_link0 下的具体坐标（详见 §8 D5）。

#### `abort_current_task()`

**输入**：无参数

**输出**：

```json
{ "status": "aborted", "what": "pick_object(red_block_01)" }
```

被中断的那个工具调用立即返回 `SERVO_ABORTED` 错误。

### 4.3 错误对象与错误码枚举

所有失败一律返回三字段结构：

```json
{
  "status": "failed",
  "code": "OBJECT_NOT_VISIBLE",
  "reason": "red_block_01 在视野外，相机最后一次见到它是 8 秒前",
  "suggestion": "调用 scan_scene 重新定位，或要求用户把物体移回工作区"
}
```

- `code` 给程序逻辑用 / `reason` 给 LLM 理解 / `suggestion` 是软提示（LLM 可不听）。
- `suggestion` 不是命令，文档级声明（详见 §8 D3）。

**错误码枚举（钉死，新增需更新本文档）**：

| code | 含义 | 典型 suggestion |
|---|---|---|
| `OBJECT_NOT_VISIBLE` | 指定 id 当前不在视野 | 重新 scan_scene |
| `UNKNOWN_OBJECT_ID` | id 在 blackboard 不存在 | scan_scene 或检查 id 拼写 |
| `OUT_OF_REACH` | 目标坐标超出工作空间 | 让用户移动物体到工作区中央 |
| `MOTION_PLANNING_FAILED` | MoveIt 规划失败 | 重试，或先 scan_scene 检查障碍 |
| `MOTION_COLLISION` | 执行中检测到碰撞 | 先 abort，再 scan_scene |
| `GRASP_PLANNING_FAILED` | 抓取位姿求解失败 | 换一个物体或换一个角度 |
| `GRIPPER_SLIPPED` | 抓取后验证检测物体已不在夹爪内 | 重新 pick_object |
| `SERVO_TIMEOUT` | 伺服环超过最大迭代次数仍未收敛 | 重新 pick_object |
| `SERVO_ABORTED` | 被 abort_current_task 中断 | 由 LLM 决定后续 |
| `INTERNAL_ERROR` | 兜底未分类异常 | 终止任务，报告用户 |

### 4.4 同步语义与 timeout

- **MCP 客户端 timeout**：60 秒。覆盖单步阻塞工具最长耗时 2 倍。
- **内环超时**：每个阻塞工具内部有自己的看门狗（如 `pick_object` 30 秒），超时返回 `SERVO_TIMEOUT`，**不让 MCP timeout 来兜底**。
- **唯一中断手段**：`abort_current_task`。LLM 在收到用户"停下"指令时主动调它。

---

## 5. 内环：ROS 2 执行层

### 5.1 视觉伺服节点（30 Hz P 控制版）

**职责**：把 `pick_object(id)` 这样的语义指令翻译成 30 Hz 的位置微调循环，直到末端对准目标 → 闭夹爪 → 抬起 → 验证。

**伪流程**：

```python
def pick_loop(object_id):
    target = blackboard.lookup(object_id)
    if target is None:
        return Error(UNKNOWN_OBJECT_ID, ...)

    iter_count = 0
    while True:
        if aborted:                      # 抢占
            stop_arm(); return Error(SERVO_ABORTED, ...)
        if iter_count > MAX_ITER:        # 看门狗
            stop_arm(); return Error(SERVO_TIMEOUT, ...)

        current_pose = perception.locate(object_id)
        if current_pose is None:
            return Error(OBJECT_NOT_VISIBLE, ...)

        err = current_pose - end_effector_pose
        if err.norm() < CONVERGE_THRESH:
            break

        moveit.move_relative(err * KP)   # KP 起步 0.3，靠跑 demo 调
        iter_count += 1

    close_gripper()
    lift(LIFT_HEIGHT)
    if not perception.verify_held(object_id):
        return Error(GRIPPER_SLIPPED, ...)
    return OK
```

**关键参数（初始值，靠 demo 调）**：

- `KP = 0.3`（P 系数）
- `CONVERGE_THRESH = 5mm`
- `MAX_ITER = 900`（30 秒 @ 30 Hz）
- `LIFT_HEIGHT = 0.15m`

> ⚠️ 待验证：MoveIt2 的 `move_relative` 在 30 Hz 调用频率下是否稳定，可能需要换成更轻量的 servo 接口（`moveit_servo`）。

### 5.2 感知节点：YOLO + 深度反投影

**职责**：订阅 RGB-D 相机，输出 `id → 6D pose` 写入 `/scene_state`。

**流程**：

1. 订阅 `/camera/color/image_raw` 与 `/camera/depth/image_raw`
2. RGB 过 YOLO（COCO 预训练 + 桌面物体微调，或 HSV 色块分割兜底）→ 得到 bbox + label
3. 对每个 bbox，取 bbox 中心像素 → 查深度图 → 反投影到 camera_optical_frame 3D 点
4. 通过 TF 变换到 panda_link0
5. 给每个稳定的检测分配持久 id（`<label>_<seq>` 命名，详见 §6.1）
6. 发布到 `/scene_state` 话题（`object_id` → 6D pose），频率 5 Hz

> ⚠️ 待验证：Gazebo 渲染分布与真实图像差距很大，YOLO 现成权重大概率认不出。回退方案：用 HSV 色块分割（红/蓝/绿块）冒充"YOLO"，报告里诚实说明。

### 5.3 状态总线：`/scene_state` blackboard

- **形式**：自定义 ROS 2 message，键 `object_id` → 值 `geometry_msgs/PoseStamped` + `last_seen_at`
- **写入**：感知节点 5 Hz
- **读取**：所有 MCP 工具内部
- **过期判定**：`last_seen_at` 超过 10 秒视为该 id 不可用 → 触发 `OBJECT_NOT_VISIBLE`

### 5.4 MoveIt2 集成

继承现有 `my_panda_moveit_config` 包。新增点：

- 把感知出的物体作为 collision objects 加入 planning scene（避碰用）
- 视觉伺服节点优先用 `moveit_servo` 接口而非 `MoveGroup` action（频率匹配）
- 现有 `mcp_panda_brain.py` 中的 `MoveGroup` 调用作为大幅运动的后备

### 5.5 安全围栏

继承 `mcp_panda_brain.py` 已有的工作空间盒：X[0.2, 0.6] / Y[-0.4, 0.4] / Z[0.2, 0.7]。**所有新工具必须在内部强制此检查**（grt-collaborating §3.1 的延伸）。

新增：

- **碰撞检测**：MoveIt 自身的碰撞检查保留，触发时返回 `MOTION_COLLISION`
- **力矩限制**：仿真层面用 ros2_control 的限位；真机阶段（不在本课设范围）才需要 libfranka 的力反馈

---

## 6. 状态与坐标系

### 6.1 `object_id` 命名约定

- 格式：`<label>_<sequence>`
- 例：`red_block_01`、`blue_plate_03`
- `sequence` 由感知节点维护，从 01 起递增
- 同一 id 在场景重置前持久有效，跨 `scan_scene` 调用稳定

### 6.2 内部坐标系：`panda_link0` 单一基准

- 所有 ROS 消息（`PoseStamped`、`PositionConstraint` 的 `frame_id`）一律 `panda_link0`
- 相机坐标系（`camera_optical_frame`）只在感知节点内部出现，不外泄
- LLM 完全不可见任何坐标系名（详见 §8 D5）

### 6.3 TF 树

```
world
  └── panda_link0
        ├── panda_link1 ... panda_link8 (机械臂链)
        │     └── panda_hand
        │           └── panda_leftfinger / panda_rightfinger
        └── camera_link
              └── camera_optical_frame
```

`world → panda_link0` 静态变换；`camera_link → panda_link0` 是手眼标定的产物，仿真里直接由 SDF 写死。

### 6.4 LLM 不可见的内部状态清单

LLM 永远看不到以下东西（看到了就是 bug）：

- 任何 (x, y, z) 浮点数
- 任何坐标系名（`panda_link0` / `camera_optical_frame` / ...）
- 任何关节角度
- TF 变换矩阵
- MoveIt planning scene 的内部表示
- ros2_control 的控制器状态

---

## 7. Gazebo 仿真环境

### 7.1 场景

- **机械臂**：Franka Panda（沿用 `my_panda_moveit_config`）
- **相机**：Gazebo RGB-D 相机插件，固定安装在桌面斜上方（eye-in-world 配置，非 eye-in-hand）
- **桌面物体**：3–5 个色块（红 / 蓝 / 绿）+ 1 个浅色盘子，作为抓取目标
- **桌面**：固定 0.4m × 0.6m 平面，z=0

### 7.2 已知风险

> ⚠️ 待验证：`franka_ros2` 在 Humble 下的 Gazebo 集成历史有坑。可能要改用 `panda_gazebo` 社区包或迁移到 Isaac Sim。**这是整个课设最阻塞的事，建议第一周就跑通"Panda 在 Gazebo 里能动 + 相机能出图"，否则后续全是空中楼阁**（详见 §9）。

---

## 8. 关键决策记录（ADR）

### D1 工具同步阻塞 + 抢占式 abort

- **选择**：MCP 工具同步返回最终结果；额外提供全局 `abort_current_task()` 中断阻塞调用。
- **理由**：异步 task_id + 轮询会让 LLM 必须在 prompt 里维护状态机，复杂度对课设是负担。同步阻塞的真正缺陷是不可中断 —— abort 通道补齐这个。
- **边界**：单步任务 < 30s 时同步合适。"边规划边执行"或长任务必须改异步。
- **升级路径**：如果将来要做并发任务（同时控制两条手臂、或边走边看），整体改异步 task_id。

### D2 状态走 ROS blackboard，LLM 上下文只见 id

- **选择**：`scan_scene` 返回语义 id，真实 6D pose 存 `/scene_state`；LLM 的工具调用之间用 id 串起来，不见浮点坐标。
- **理由**：让 LLM 在 prompt 里搬运浮点字符串会丢精度、token 贵、易抄错。id 解耦 LLM 与坐标细节，将来换坐标系不影响 prompt。
- **边界**：LLM 失去几何推理能力（"放在两个杯子中点"做不了）。
- **升级路径**：需要几何推理时加只读工具 `query_object_pose(id)`，把口子开得最小。

### D3 错误对象三字段（code / reason / suggestion）

- **选择**：所有失败返回结构化对象，含枚举 code、自然语言 reason、软提示 suggestion。
- **理由**：纯 "失败" 没信息量，LLM 只能瞎重试；纯自然语言 LLM 解析不稳定。三字段各司其职：code 给程序、reason 给理解、suggestion 帮弱模型走对路。
- **边界**：suggestion 是软提示，LLM 可以不听。错误码枚举一旦发布就要稳定，新增需更新本文档。
- **升级路径**：错误码不够用就加，但**不能改语义**。

### D4 夹爪封装在 pick / place 内，set_gripper 仅 debug

- **选择**：`pick_object` / `place_at` 内部完成夹爪开合时序；`set_gripper` 保留但不入 LLM 默认 prompt。
- **理由**：暴露 set_gripper 给 LLM = 让它推理"先到位再闭合"这种隐含序列，它会出错。把动作单元抬到"抓 / 放"层级，LLM 推理负担显著降低。
- **边界**：失去"半开夹爪推一下物体"这类创意操作。课设范围内不需要。
- **升级路径**：将来需要复杂操作时把 `set_gripper` 加入 LLM prompt，但要补充安全前置条件。

### D5 LLM 不可见坐标系，内部统一 `panda_link0`

- **选择**：对 LLM 只暴露语义位置（`above` / `left_of` / `between` 等枚举）；ROS 内部所有消息一律 `panda_link0`。
- **理由**：让 LLM 理解多坐标系是灾难，连大模型也会偶尔搞混 optical_frame 方向。课设场景固定，语义位置够用。
- **边界**：失去任意 6D 位姿放置能力。
- **升级路径**：扩展 `offset` 枚举（`between(a, b)` 等），或加 `query_object_pose(id)` 让 LLM 算几何，但仍不暴露坐标系名。

---

## 9. Scope 风险与已知未解

| 风险 | 影响 | 状态 |
|---|---|---|
| `franka_ros2` + Gazebo + Humble 集成 | **最阻塞**，不通则全盘空中楼阁 | 待验证，第一周必须验 |
| MCP 客户端 long-call timeout 撞 60s 上限 | 单次抓取异常长可能整工具失败 | 缓解：内环看门狗 30s 兜底；如仍不够则 D1 改异步 |
| Gazebo 渲染分布下 YOLO 泛化 | 现成权重大概率认不出 | 缓解：HSV 色块分割兜底，报告里说明 |
| Whisper 本地推理资源占用 | 显存 / CPU 不够导致延迟过高 | 缓解：降级到 `tiny` 模型；极端情况转用云端 API |
| `moveit_servo` 在 30 Hz 下的稳定性 | 伺服循环抖动或失稳 | 缓解：从 10 Hz 起步逐步加频；KP 保守 |
| LLM 工具调用 hallucination（传错 id） | 内环报错而不是默默错执行 | 已设计：`UNKNOWN_OBJECT_ID` 显式拦截 |

---

## 10. 与 CLAUDE.md 的关系

`CLAUDE.md` 是**运维与命令指南**：怎么起容器、怎么 build、stdio 隔离规则、MoveIt action 名等运行时纪律。`DESIGN.md`（本文档）是**架构与接口契约**：分层、工具 schema、错误码、决策记录。两者不重叠，遇到冲突以本文档对架构问题、`CLAUDE.md` 对运维问题各自为准。
