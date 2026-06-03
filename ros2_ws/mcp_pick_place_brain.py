"""
Unified MCP server for pick-and-place (DESIGN.md §4).

Tools exposed to LLM: scan_scene, pick_object, place_at, abort_current_task
Debug tools: execute_arm_move, set_gripper
"""

from __future__ import annotations

import os
import sys
import threading

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
from panda_pick_place.offset_resolver import VALID_OFFSETS  # noqa: E402
from panda_pick_place.workspace_envelope import DEFAULT_ENVELOPE  # noqa: E402
from panda_pick_place.gripper_helper import GripperHelper  # noqa: E402
from panda_pick_place.moveit_helper import MoveItHelper  # noqa: E402
from panda_pick_place.mcp_validation import validate_pick_target, validate_place_target  # noqa: E402


ACTION_TIMEOUT_SEC = 180.0


class PickPlaceMcpBridge(Node):
    def __init__(self) -> None:
        super().__init__("mcp_pick_place_bridge")

        self._latest_scene: SceneState | None = None
        self.create_subscription(SceneState, "/scene_state", self._on_scene, 10)

        self._scan_client = self.create_client(TriggerScan, "/perception/trigger_scan")
        self._abort_client = self.create_client(AbortTask, "/pick_place/abort")
        self._pick_client = ActionClient(self, PickObject, "/pick_place/pick_object")
        self._place_client = ActionClient(self, PlaceAt, "/pick_place/place_at")
        self._moveit = MoveItHelper(self)
        self._gripper = GripperHelper(self)

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

    def scan_scene(self) -> str:
        refreshed = self._call_scan()
        if not refreshed and self._latest_scene is None:
            return failed(ErrorCode.INTERNAL_ERROR, "感知节点未就绪，请先启动 pick_place.launch.py")

        objects = []
        if self._latest_scene:
            for obj in self._latest_scene.objects:
                objects.append({
                    "id": obj.id,
                    "label": obj.label,
                    "confidence": round(float(obj.confidence), 2),
                })

        return ok(objects=objects, scanned_at=utc_now_iso())

    def pick_object(self, object_id: str) -> str:
        if not self._pick_client.server_is_ready():
            return failed(ErrorCode.INTERNAL_ERROR, "pick_place 执行器未就绪")

        self._call_scan()
        pre = validate_pick_target(self._latest_scene, object_id)
        if pre is not None:
            return pre

        goal = PickObject.Goal()
        goal.object_id = object_id
        send_future = self._pick_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, timeout_sec=5.0)
        if goal_handle is None or not goal_handle.accepted:
            return failed(ErrorCode.INTERNAL_ERROR, "无法提交 pick_object 目标")

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(result_future, timeout_sec=ACTION_TIMEOUT_SEC)
        if wrapped is None:
            return failed(ErrorCode.SERVO_TIMEOUT, f"pick_object({object_id}) 超时")

        result = wrapped.result
        if result.success:
            return ok(id=object_id)
        return failed(
            result.code or ErrorCode.INTERNAL_ERROR.value,
            result.reason or "pick 失败",
            result.suggestion or None,
        )

    def place_at(self, target_id: str, offset: str) -> str:
        if offset not in VALID_OFFSETS:
            return failed(
                ErrorCode.INTERNAL_ERROR,
                f"offset 必须是 {sorted(VALID_OFFSETS)} 之一",
            )

        if not self._place_client.server_is_ready():
            return failed(ErrorCode.INTERNAL_ERROR, "pick_place 执行器未就绪")

        if target_id == "outside_table":
            return failed(
                ErrorCode.OUT_OF_REACH,
                "放置目标超出安全工作空间（演示：桌子外面）",
            )

        self._call_scan()
        pre = validate_place_target(self._latest_scene, target_id, offset)
        if pre is not None:
            return pre

        goal = PlaceAt.Goal()
        goal.target_id = target_id
        goal.offset = offset
        send_future = self._place_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, timeout_sec=5.0)
        if goal_handle is None or not goal_handle.accepted:
            return failed(ErrorCode.INTERNAL_ERROR, "无法提交 place_at 目标")

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(result_future, timeout_sec=ACTION_TIMEOUT_SEC)
        if wrapped is None:
            return failed(ErrorCode.SERVO_TIMEOUT, f"place_at({target_id}, {offset}) 超时")

        result = wrapped.result
        if result.success:
            return ok(target_id=target_id, offset=offset)
        return failed(
            result.code or ErrorCode.INTERNAL_ERROR.value,
            result.reason or "place 失败",
            result.suggestion or None,
        )

    def abort_current_task(self) -> str:
        if not self._abort_client.wait_for_service(timeout_sec=0.5):
            return failed(ErrorCode.INTERNAL_ERROR, "abort 服务不可用")
        future = self._abort_client.call_async(AbortTask.Request())
        result = self._wait_future(future, timeout_sec=2.0)
        if result is None:
            return failed(ErrorCode.INTERNAL_ERROR, "abort 调用超时")
        return aborted(result.what_was_aborted or "unknown")

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


mcp = FastMCP("Panda_PickPlace")
ros_node: PickPlaceMcpBridge | None = None


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
