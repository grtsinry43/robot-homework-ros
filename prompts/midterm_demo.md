# 中期展示：MCP 工具链演示手册

> 核心叙事：**LLM 理解任务 → MCP 约束工具调用 → ROS 2 / MoveIt2 感知与执行**  
> LLM **不输出坐标**，只选 `id` 和语义 `offset`。

## 一、演示前准备（约 5 分钟）

```bash
# 宿主机
./scripts/run_in_container.sh stop
./scripts/run_in_container.sh start-phase2-unified   # Gazebo + MoveIt + 感知 + 执行器
./scripts/run_in_container.sh verify-phase2
```

可选：Cursor 接 MCP（现场对话抓放）

```bash
# 复制 config/mcp_client.docker.json 到 Cursor MCP 设置（改 compose 路径）
# 确保容器在跑且 ROS 栈已起；MCP 由 docker exec 启动 mcp_pick_place_brain.py
```

**无 Cursor 时**（推荐彩排）：用演示 CLI，工具 JSON 与 MCP 完全一致。

```bash
docker compose exec ros2-gazebo bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
python3 /root/scripts/mcp_demo_cli.py scenario 1 --execute
```

---

## 二、五段展示顺序

### 1. 基础语义抓放（主链路）

**用户说：** `把红色方块放到蓝色盘子里`

**工具链：**

```text
scan_scene()
pick_object(id="red_block_01")
place_at(target_id="blue_plate_01", offset="above")
```

**演示命令：**

```bash
python3 /root/scripts/mcp_demo_cli.py nl "把红色方块放到蓝色盘子里" --execute
# 或
python3 /root/scripts/mcp_demo_cli.py scenario 1 --execute
```

**讲解要点：** id 来自感知 blackboard；MCP 返回 JSON，无 `(x,y,z)`。

---

### 2. 同义表达鲁棒性

**示例指令：**

- 把红色的那个放进盘子
- 把红块放到蓝盘上

**演示命令：**

```bash
python3 /root/scripts/mcp_demo_cli.py scenario 2
```

**讲解要点：** `mcp_intent.py` 模拟 LLM 归一化；真实验收接 LLM + `prompts/llm_system_prompt.md`。

---

### 3. 参数与安全检查

**示例与期望错误：**

| 用户意图 | 工具调用 | 期望 `code` |
|---------|----------|-------------|
| 把不存在的紫色球… | `pick_object("purple_ball_01")` | `UNKNOWN_OBJECT_ID` |
| 非法 offset | `place_at(..., "diagonal")` | 参数校验失败 |
| 放到桌子外面 | `place_at("outside_table", …)` | `OUT_OF_REACH` |
| 把它放过去 | （无明确目标） | LLM 澄清，不盲目 `place_at` |

**演示命令：**

```bash
python3 /root/scripts/mcp_demo_cli.py scenario 3
```

**讲解要点：** MCP 层 schema + 物体存在性 + 工作空间 envelope；不是 LLM 裸控关节。

---

### 4. 黄色障碍物避障（设计 + 可见场景）

**场景：** Gazebo 桌面有 `yellow_wall_01`（红块与蓝盘之间）。

**逻辑说明：**

```text
scan_scene() → red_block_01 / blue_plate_01 / (yellow_wall 可见)
pick_object(red_block_01)
place_at(blue_plate_01, right_of)
MoveIt2 planning scene 中加入障碍 → 几何避障（中期：方案 + RViz 演示）
```

**演示命令：**

```bash
python3 /root/scripts/mcp_demo_cli.py scenario 4
python3 /root/scripts/mcp_demo_cli.py scenario 4 --execute   # 可选真机移动
```

**讲解要点：** 黄墙 **不由 LLM 规划路径**；后续工作：感知障碍 → 写入 planning scene → `MOTION_COLLISION` 上报。

---

### 5. 语音输入（扩展）

**演示命令：**

```bash
python3 /root/scripts/resolve_user_input.py --mode text --text "把红色方块放到蓝色盘子里"
python3 /root/scripts/mcp_demo_cli.py scenario 5 --execute
```

**讲解要点：** 语音只是输入通道；核心仍是 MCP 工具调用。

---

## 三、一键彩排

```bash
# 容器内，仅打印计划（不运动）
bash /root/scripts/demo_midterm_mcp.sh

# 场景1 真抓放
bash /root/scripts/demo_midterm_mcp.sh --execute 1

# 全部场景（分段暂停）
bash /root/scripts/demo_midterm_mcp.sh --execute all
```

---

## 四、MCP 工具 JSON 示例

**scan_scene 成功：**

```json
{"status":"ok","objects":[{"id":"red_block_01","label":"red_block","confidence":0.72}],"scanned_at":"..."}
```

**失败：**

```json
{"status":"failed","code":"UNKNOWN_OBJECT_ID","reason":"blackboard 无 purple_ball_01","suggestion":"调用 scan_scene 或检查 id 拼写"}
```

---

## 五、分工对照（答辩用）

| 层级 | 职责 |
|------|------|
| LLM | 自然语言 → 工具名 + 参数（id / offset） |
| MCP | JSON schema、前置校验、结构化错误 |
| ROS 感知 | HSV + 深度 → `/scene_state` |
| ROS 执行 | MoveIt 宏观 + executor 内环 |
| MoveIt2 | 轨迹规划、碰撞几何（避障目标） |
