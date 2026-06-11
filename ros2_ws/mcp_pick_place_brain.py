"""
Unified MCP server for pick-and-place (DESIGN.md §4).

Tools exposed to LLM: get_action_library, get_robot_context, execute_plan,
scan_scene, pick_object, place_at, place_object, abort_current_task
Debug tools: execute_arm_move, set_gripper
"""

from __future__ import annotations

import os
import sys
import threading
import json
import re

os.environ["RCUTILS_LOGGING_USE_STDERR"] = "1"

import rclpy
from mcp.server.fastmcp import FastMCP
from pick_place_msgs.action import PickObject, PlaceAt
from pick_place_msgs.srv import AbortTask, TriggerScan
from rclpy.action import ActionClient
from rclpy.node import Node
from scene_state_msgs.msg import SceneState

# Allow importing shared helpers from the ament package when run as a script.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.join(_SCRIPT_DIR, "src", "panda_pick_place")
if os.path.isdir(_PKG_ROOT) and _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from panda_pick_place.errors import (  # noqa: E402
    ErrorCode,
    aborted,
    failed,
    ok,
    utc_now_iso,
)
from panda_pick_place.action_library import load_action_library, render_action_library  # noqa: E402
from panda_pick_place.offset_resolver import VALID_OFFSETS  # noqa: E402
from panda_pick_place.workspace_envelope import DEFAULT_ENVELOPE  # noqa: E402
from panda_pick_place.gripper_helper import GripperHelper  # noqa: E402
from panda_pick_place.moveit_helper import MoveItHelper  # noqa: E402
from panda_pick_place.mcp_validation import validate_pick_target, validate_place_target  # noqa: E402
from panda_pick_place.plan_executor import (  # noqa: E402
    PlanStep,
    PlanValidationError,
    parse_plan,
)
from panda_pick_place.robot_context import RobotContextBuilder  # noqa: E402


ACTION_TIMEOUT_SEC = 180.0
MAX_RECENT_EVENTS = 10
TOKEN_SPLIT_PATTERN = r"[^a-z0-9]+"


class PickPlaceMcpBridge(Node):
    def __init__(self) -> None:
        super().__init__("mcp_pick_place_bridge")

        self._latest_scene: SceneState | None = None
        self._active_task: str | None = None
        self._last_error: dict | None = None
        self._recent_events: list[dict] = []
        self.create_subscription(SceneState, "/scene_state", self._on_scene, 10)

        self._scan_client = self.create_client(TriggerScan, "/perception/trigger_scan")
        self._abort_client = self.create_client(AbortTask, "/pick_place/abort")
        self._pick_client = ActionClient(self, PickObject, "/pick_place/pick_object")
        self._place_client = ActionClient(self, PlaceAt, "/pick_place/place_at")
        self._moveit = MoveItHelper(self)
        self._gripper = GripperHelper(self)
        self._context = RobotContextBuilder(self)

        self.get_logger().info("MCP pick-place bridge ready")

    def _on_scene(self, msg: SceneState) -> None:
        self._latest_scene = msg

    def _wait_future(self, future, timeout_sec: float = 5.0):
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and not future.done():
            if self.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def _call_scan(self) -> bool:
        if not self._scan_client.wait_for_service(timeout_sec=0.5):
            return False
        future = self._scan_client.call_async(TriggerScan.Request())
        result = self._wait_future(future, timeout_sec=3.0)
        return bool(result and result.success)

    def _resolve_scene_object_id(self, object_ref: str) -> str | None:
        """Resolve a scene object reference to a concrete object id.

        Resolution order:
        1) exact match on object id or label;
        2) token match (split by non [a-z0-9]) on id/label.
        Returns None when no object matches or multiple objects match fuzzily.
        """
        ref = object_ref.strip()
        if not ref or self._latest_scene is None:
            return None
        normalized = ref.casefold()
        normalized_parts = {part for part in re.split(TOKEN_SPLIT_PATTERN, normalized) if part}
        if not normalized_parts:
            return None
        exact: str | None = None
        fuzzy_matches: set[str] = set()
        for obj in self._latest_scene.objects:
            obj_id = obj.id.strip()
            label = obj.label.strip()
            id_norm = obj_id.casefold()
            label_norm = label.casefold()
            if normalized == id_norm or normalized == label_norm:
                exact = obj_id
                break
            id_parts = {part for part in re.split(TOKEN_SPLIT_PATTERN, id_norm) if part}
            label_parts = {part for part in re.split(TOKEN_SPLIT_PATTERN, label_norm) if part}
            if normalized_parts & id_parts or normalized_parts & label_parts:
                fuzzy_matches.add(obj_id)
        if exact is not None:
            return exact
        if len(fuzzy_matches) != 1:
            return None
        return next(iter(fuzzy_matches))

    def scan_scene(self) -> str:
        refreshed = self._call_scan()
        if not refreshed and self._latest_scene is None:
            return self._record_tool_result(
                "scan_scene",
                failed(ErrorCode.INTERNAL_ERROR, "感知节点未就绪，请先启动 pick_place.launch.py"),
            )

        objects = []
        if self._latest_scene:
            for obj in self._latest_scene.objects:
                objects.append({
                    "id": obj.id,
                    "label": obj.label,
                    "confidence": round(float(obj.confidence), 2),
                })

        return self._record_tool_result("scan_scene", ok(objects=objects, scanned_at=utc_now_iso()))

    def pick_object(self, object_id: str) -> str:
        if not self._pick_client.server_is_ready():
            return self._record_tool_result(
                "pick_object",
                failed(ErrorCode.INTERNAL_ERROR, "pick_place 执行器未就绪"),
            )

        self._call_scan()
        resolved_id = self._resolve_scene_object_id(object_id)
        if resolved_id is None:
            self.get_logger().warning(f"pick_object: unresolved ref '{object_id}', using literal ID")
            resolved_id = object_id
        pre = validate_pick_target(self._latest_scene, resolved_id)
        if pre is not None:
            return self._record_tool_result("pick_object", pre)

        goal = PickObject.Goal()
        goal.object_id = resolved_id
        self._active_task = f"pick_object({resolved_id})"
        try:
            send_future = self._pick_client.send_goal_async(goal)
            goal_handle = self._wait_future(send_future, timeout_sec=5.0)
            if goal_handle is None or not goal_handle.accepted:
                return self._record_tool_result(
                    "pick_object",
                    failed(ErrorCode.INTERNAL_ERROR, "无法提交 pick_object 目标"),
                )

            result_future = goal_handle.get_result_async()
            wrapped = self._wait_future(result_future, timeout_sec=ACTION_TIMEOUT_SEC)
            if wrapped is None:
                return self._record_tool_result(
                    "pick_object",
                    failed(ErrorCode.SERVO_TIMEOUT, f"pick_object({resolved_id}) 超时"),
                )

            result = wrapped.result
            if result.success:
                return self._record_tool_result("pick_object", ok(id=resolved_id))
            return self._record_tool_result(
                "pick_object",
                failed(
                    result.code or ErrorCode.INTERNAL_ERROR.value,
                    result.reason or "pick 失败",
                    result.suggestion or None,
                ),
            )
        finally:
            self._active_task = None

    def place_at(self, target_id: str, offset: str) -> str:
        if offset not in VALID_OFFSETS:
            return self._record_tool_result(
                "place_at",
                failed(
                    ErrorCode.INTERNAL_ERROR,
                    f"offset 必须是 {sorted(VALID_OFFSETS)} 之一",
                ),
            )

        if not self._place_client.server_is_ready():
            return self._record_tool_result(
                "place_at",
                failed(ErrorCode.INTERNAL_ERROR, "pick_place 执行器未就绪"),
            )

        if target_id == "outside_table":
            return self._record_tool_result(
                "place_at",
                failed(
                    ErrorCode.OUT_OF_REACH,
                    "放置目标超出安全工作空间（演示：桌子外面）",
                ),
            )

        self._call_scan()
        resolved_target_id = self._resolve_scene_object_id(target_id)
        if resolved_target_id is None:
            self.get_logger().warning(f"place_at: unresolved ref '{target_id}', using literal ID")
            resolved_target_id = target_id
        pre = validate_place_target(self._latest_scene, resolved_target_id, offset)
        if pre is not None:
            return self._record_tool_result("place_at", pre)

        goal = PlaceAt.Goal()
        goal.target_id = resolved_target_id
        goal.offset = offset
        self._active_task = f"place_at({resolved_target_id}, {offset})"
        try:
            send_future = self._place_client.send_goal_async(goal)
            goal_handle = self._wait_future(send_future, timeout_sec=5.0)
            if goal_handle is None or not goal_handle.accepted:
                return self._record_tool_result(
                    "place_at",
                    failed(ErrorCode.INTERNAL_ERROR, "无法提交 place_at 目标"),
                )

            result_future = goal_handle.get_result_async()
            wrapped = self._wait_future(result_future, timeout_sec=ACTION_TIMEOUT_SEC)
            if wrapped is None:
                return self._record_tool_result(
                    "place_at",
                    failed(ErrorCode.SERVO_TIMEOUT, f"place_at({resolved_target_id}, {offset}) 超时"),
                )

            result = wrapped.result
            if result.success:
                return self._record_tool_result("place_at", ok(target_id=resolved_target_id, offset=offset))
            return self._record_tool_result(
                "place_at",
                failed(
                    result.code or ErrorCode.INTERNAL_ERROR.value,
                    result.reason or "place 失败",
                    result.suggestion or None,
                ),
            )
        finally:
            self._active_task = None

    def abort_current_task(self) -> str:
        if not self._abort_client.wait_for_service(timeout_sec=0.5):
            return self._record_tool_result(
                "abort_current_task",
                failed(ErrorCode.INTERNAL_ERROR, "abort 服务不可用"),
            )
        future = self._abort_client.call_async(AbortTask.Request())
        result = self._wait_future(future, timeout_sec=2.0)
        if result is None:
            return self._record_tool_result(
                "abort_current_task",
                failed(ErrorCode.INTERNAL_ERROR, "abort 调用超时"),
            )
        return self._record_tool_result(
            "abort_current_task",
            aborted(result.what_was_aborted or "unknown"),
        )

    def set_gripper(self, state: str) -> str:
        if state not in {"open", "close"}:
            return failed(ErrorCode.INTERNAL_ERROR, "state 必须是 open 或 close")
        if not self._gripper.ready():
            return failed(ErrorCode.INTERNAL_ERROR, "franka_gripper action 未就绪")
        if state == "open":
            success, code, reason = self._gripper.open()
        else:
            success, code, reason = self._gripper.close()
        if success:
            return ok(gripper=state)
        return failed(code, reason)

    def execute_arm_move(self, x: float, y: float, z: float) -> str:
        err = DEFAULT_ENVELOPE.check_or_error(x, y, z)
        if err:
            return err

        success, code, reason = self._moveit.move_to_xyz(float(x), float(y), float(z))
        if success:
            return ok(x=x, y=y, z=z)
        return failed(code, reason)

    def get_robot_context(self) -> str:
        context = self._context.build(
            scene=self._latest_scene,
            readiness={
                "perception_service": self._scan_client.wait_for_service(timeout_sec=0.0),
                "abort_service": self._abort_client.wait_for_service(timeout_sec=0.0),
                "pick_action": self._pick_client.server_is_ready(),
                "place_action": self._place_client.server_is_ready(),
                "moveit_action": self._moveit.server_ready(),
                "gripper_actions": self._gripper.ready(),
            },
            active_task=self._active_task,
            last_error=self._last_error,
            recent_events=list(self._recent_events),
        )
        return ok(context=context)

    def execute_plan(self, plan_json: str) -> str:
        try:
            steps = parse_plan(plan_json, load_action_library())
        except PlanValidationError as exc:
            return self._record_tool_result(
                "execute_plan",
                failed(ErrorCode.INTERNAL_ERROR, f"invalid plan: {exc}"),
            )
        except Exception as exc:  # noqa: BLE001
            return self._record_tool_result(
                "execute_plan",
                failed(ErrorCode.INTERNAL_ERROR, f"unable to prepare plan: {exc}"),
            )

        step_results: list[dict] = []
        for index, step in enumerate(steps):
            raw_result = self._execute_plan_step(step)
            payload = self._parse_tool_payload(raw_result)
            step_results.append(
                {
                    "index": index,
                    "tool": step.tool,
                    "args": step.args,
                    "result": payload,
                }
            )

            status = payload.get("status")
            if status == "failed":
                return self._record_tool_result(
                    "execute_plan",
                    self._plan_failed_response(index, step, payload, step_results),
                )
            if status == "aborted":
                return self._record_tool_result(
                    "execute_plan",
                    self._plan_aborted_response(index, step, payload, step_results),
                )

        return self._record_tool_result(
            "execute_plan",
            ok(
                plan_status="completed",
                steps=step_results,
                final_context=self._current_context_payload(),
            ),
        )

    def _execute_plan_step(self, step: PlanStep) -> str:
        if step.tool == "get_robot_context":
            return self.get_robot_context()
        if step.tool == "scan_scene":
            return self.scan_scene()
        if step.tool == "pick_object":
            return self.pick_object(step.args["id"])
        if step.tool == "place_at":
            return self.place_at(step.args["target_id"], step.args["offset"])
        if step.tool == "abort_current_task":
            return self.abort_current_task()
        return failed(ErrorCode.INTERNAL_ERROR, f"unsupported plan step: {step.tool}")

    def _parse_tool_payload(self, raw_result: str) -> dict:
        try:
            payload = json.loads(raw_result)
        except json.JSONDecodeError:
            return {
                "status": "failed",
                "code": ErrorCode.INTERNAL_ERROR.value,
                "reason": "tool returned non-JSON output",
                "suggestion": "停止任务，检查 MCP tool implementation",
            }
        if not isinstance(payload, dict):
            return {
                "status": "failed",
                "code": ErrorCode.INTERNAL_ERROR.value,
                "reason": "tool returned a non-object JSON payload",
                "suggestion": "停止任务，检查 MCP tool implementation",
            }
        return payload

    def _plan_failed_response(
        self,
        index: int,
        step: PlanStep,
        payload: dict,
        step_results: list[dict],
    ) -> str:
        code = payload.get("code") or ErrorCode.INTERNAL_ERROR.value
        reason = payload.get("reason") or f"{step.tool} failed"
        suggestion = payload.get("suggestion") or "调用 get_robot_context 检查 recent_events 后决定是否重试"
        return json.dumps(
            {
                "status": "failed",
                "code": code,
                "reason": f"plan stopped at step {index} ({step.tool}): {reason}",
                "suggestion": suggestion,
                "plan_status": "stopped_on_failed_step",
                "failed_step_index": index,
                "steps": step_results,
                "final_context": self._current_context_payload(),
            },
            ensure_ascii=False,
        )

    def _plan_aborted_response(
        self,
        index: int,
        step: PlanStep,
        payload: dict,
        step_results: list[dict],
    ) -> str:
        what = payload.get("what") or step.tool
        return json.dumps(
            {
                "status": "aborted",
                "what": what,
                "plan_status": "stopped_on_aborted_step",
                "failed_step_index": index,
                "steps": step_results,
                "final_context": self._current_context_payload(),
            },
            ensure_ascii=False,
        )

    def _current_context_payload(self) -> dict:
        payload = self._parse_tool_payload(self.get_robot_context())
        if payload.get("status") == "ok":
            context = payload.get("context")
            if isinstance(context, dict):
                return context
        return payload

    def _record_tool_result(self, tool: str, result_json: str) -> str:
        try:
            payload = json.loads(result_json)
        except json.JSONDecodeError:
            return result_json

        event = {
            "tool": tool,
            "status": payload.get("status", "unknown"),
            "recorded_at": utc_now_iso(),
        }
        if "code" in payload:
            event["code"] = payload.get("code")
        if "reason" in payload:
            event["reason"] = payload.get("reason")
        if "suggestion" in payload:
            event["suggestion"] = payload.get("suggestion")
        if "what" in payload:
            event["what"] = payload.get("what")
        self._recent_events.append(event)
        self._recent_events = self._recent_events[-MAX_RECENT_EVENTS:]

        if payload.get("status") == "failed":
            self._last_error = {
                "tool": tool,
                "code": payload.get("code"),
                "reason": payload.get("reason"),
                "suggestion": payload.get("suggestion"),
                "recorded_at": utc_now_iso(),
            }
        return result_json


mcp = FastMCP("Panda_PickPlace")
ros_node: PickPlaceMcpBridge | None = None


@mcp.tool()
def get_action_library() -> str:
    """返回当前机器人可调用能力目录（atomic action library）。"""
    try:
        library = load_action_library()
    except Exception as exc:  # noqa: BLE001
        return failed(ErrorCode.INTERNAL_ERROR, f"无法读取 action library: {exc}")
    return ok(
        schema_version=library["schema_version"],
        library_name=library["library_name"],
        actions=library["actions"],
        rendered=render_action_library(library),
    )


@mcp.tool()
def get_robot_context() -> str:
    """返回当前场景、依赖 readiness、工作空间和建议下一步。"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.get_robot_context()


@mcp.tool()
def execute_plan(plan_json: str) -> str:
    """校验并执行短线性计划。plan_json 是 [{"tool": "...", "args": {...}}] 的 JSON 字符串。"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.execute_plan(plan_json)


@mcp.tool()
def scan_scene() -> str:
    """扫描桌面场景，返回可见物体的 id 列表（不含坐标）。"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.scan_scene()


@mcp.tool()
def pick_object(id: str) -> str:
    """抓起指定 id 的物体。阻塞直到成功或失败。"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.pick_object(id)


@mcp.tool()
def place_at(target_id: str, offset: str) -> str:
    """把手中物体放到 target_id 的语义位置。offset: above/left_of/right_of/front_of/behind"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.place_at(target_id, offset)


@mcp.tool()
def place_object(id: str) -> str:
    """兼容别名：把手中物体放到目标物体上方。等价于 place_at(target_id=id, offset='above')."""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.place_at(target_id=id, offset="above")


@mcp.tool()
def abort_current_task() -> str:
    """抢占中断当前正在执行的 pick/place。"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.abort_current_task()


@mcp.tool()
def set_gripper(state: str) -> str:
    """[debug] 直接开/合夹爪。state: open | close"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.set_gripper(state)


@mcp.tool()
def execute_arm_move(x: float, y: float, z: float) -> str:
    """[debug] 移动末端到 (x,y,z)，单位米，panda_link0 坐标系。"""
    if ros_node is None:
        return failed(ErrorCode.INTERNAL_ERROR, "ROS 节点未初始化")
    return ros_node.execute_arm_move(x, y, z)


def spin_ros() -> None:
    global ros_node
    rclpy.init()
    ros_node = PickPlaceMcpBridge()
    rclpy.spin(ros_node)


if __name__ == "__main__":
    thread = threading.Thread(target=spin_ros, daemon=True)
    thread.start()
    mcp.run()
