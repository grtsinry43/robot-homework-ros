"""Pick/place inner loop — visual servo + gripper (DESIGN.md §5.1)."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from pick_place_msgs.action import PickObject, PlaceAt
from pick_place_msgs.srv import AbortTask
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scene_state_msgs.msg import SceneState
from tf2_ros import Buffer, TransformException, TransformListener

from .errors import ErrorCode, DEFAULT_SUGGESTIONS
from .gripper_helper import GripperHelper
from .moveit_helper import MoveItHelper
from .offset_resolver import VALID_OFFSETS, resolve_offset
from .servo_helper import ServoHelper
from .workspace_envelope import DEFAULT_ENVELOPE


class ExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_executor")

        self.declare_parameter("planning_frame", "panda_link0")
        self.declare_parameter("ee_link", "panda_link8")
        self.declare_parameter("move_group", "panda_arm")
        self.declare_parameter("servo_rate_hz", 30.0)
        self.declare_parameter("kp", 0.3)
        self.declare_parameter("converge_thresh_m", 0.005)
        self.declare_parameter("max_iter", 900)
        self.declare_parameter("approach_height_m", 0.12)
        self.declare_parameter("lift_height_m", 0.15)
        self.declare_parameter("stale_after_sec", 10.0)
        self.declare_parameter("servo_node_name", "servo_node")
        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("gripper_move_action", "/franka_gripper/move")
        self.declare_parameter("gripper_grasp_action", "/franka_gripper/grasp")
        self.declare_parameter("grasp_width_m", 0.035)
        self.declare_parameter("grasp_speed", 0.1)
        self.declare_parameter("grasp_force", 5.0)
        self.declare_parameter("gripper_open_width_m", 0.08)
        self.declare_parameter("allow_gripper_skip", False)

        self._cb_group = ReentrantCallbackGroup()
        self._abort = threading.Event()
        self._current_task = ""
        self._scene: SceneState | None = None
        self._held_object_id: str | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._moveit = MoveItHelper(
            self,
            group_name=self.get_parameter("move_group").value,
            ee_link=self.get_parameter("ee_link").value,
        )
        self._servo = ServoHelper(
            self,
            servo_node_name=self.get_parameter("servo_node_name").value,
            twist_topic=self.get_parameter("servo_twist_topic").value,
            planning_frame=self.get_parameter("planning_frame").value,
        )
        self._gripper = GripperHelper(
            self,
            move_action=self.get_parameter("gripper_move_action").value,
            grasp_action=self.get_parameter("gripper_grasp_action").value,
            open_width_m=self.get_parameter("gripper_open_width_m").value,
            grasp_width_m=self.get_parameter("grasp_width_m").value,
            grasp_speed=self.get_parameter("grasp_speed").value,
            grasp_force=self.get_parameter("grasp_force").value,
        )

        self.create_subscription(SceneState, "/scene_state", self._on_scene, 10)
        self.create_service(AbortTask, "/pick_place/abort", self._handle_abort, callback_group=self._cb_group)

        self._pick_server = ActionServer(
            self, PickObject, "/pick_place/pick_object",
            execute_callback=self._execute_pick,
            goal_callback=self._goal_ok, cancel_callback=self._cancel_ok,
            callback_group=self._cb_group,
        )
        self._place_server = ActionServer(
            self, PlaceAt, "/pick_place/place_at",
            execute_callback=self._execute_place,
            goal_callback=self._goal_ok, cancel_callback=self._cancel_ok,
            callback_group=self._cb_group,
        )

        self.get_logger().info("pick_place_executor ready (moveit_servo + MoveIt macro + franka_gripper)")

    def _on_scene(self, msg: SceneState) -> None:
        self._scene = msg

    def _goal_ok(self, _goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_ok(self, _goal) -> CancelResponse:
        self._abort.set()
        self._servo.stop()
        return CancelResponse.ACCEPT

    def _handle_abort(self, _request: AbortTask.Request, response: AbortTask.Response) -> AbortTask.Response:
        what = self._current_task
        self._abort.set()
        self._servo.stop()
        response.success = True
        response.what_was_aborted = what or "idle"
        return response

    def _lookup(self, object_id: str) -> PoseStamped | None:
        if self._scene is None:
            return None
        stale_ns = int(self.get_parameter("stale_after_sec").value * 1e9)
        now = self.get_clock().now()
        for obj in self._scene.objects:
            if obj.id != object_id:
                continue
            last_seen = rclpy.time.Time.from_msg(obj.last_seen_at)
            if (now - last_seen).nanoseconds > stale_ns:
                return None
            return obj.pose
        return None

    def _ee_position(self) -> tuple[float, float, float] | None:
        frame = self.get_parameter("planning_frame").value
        ee = self.get_parameter("ee_link").value
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                frame, ee, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warning(f"TF {frame}<-{ee}: {exc}")
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    @staticmethod
    def _make_error_result(result, code: ErrorCode, reason: str):
        result.success = False
        result.code = code.value
        result.reason = reason
        result.suggestion = DEFAULT_SUGGESTIONS.get(code, DEFAULT_SUGGESTIONS[ErrorCode.INTERNAL_ERROR])
        return result

    def _check_abort(self) -> bool:
        return self._abort.is_set()

    def _allow_gripper_skip(self) -> bool:
        return bool(self.get_parameter("allow_gripper_skip").value)

    def _ensure_gripper(self) -> tuple[bool, ErrorCode, str]:
        if self._gripper.ready():
            return True, ErrorCode.INTERNAL_ERROR, ""
        if self._allow_gripper_skip():
            self.get_logger().warn("夹爪未就绪，allow_gripper_skip=true — 跳过夹爪动作")
            return True, ErrorCode.INTERNAL_ERROR, ""
        return False, ErrorCode.INTERNAL_ERROR, "夹爪 action 未就绪"

    def _gripper_open(self) -> tuple[bool, ErrorCode, str]:
        if not self._gripper.ready():
            if self._allow_gripper_skip():
                return True, ErrorCode.INTERNAL_ERROR, ""
            return False, ErrorCode.INTERNAL_ERROR, "夹爪 action 未就绪"
        return self._gripper.open()

    def _gripper_close(self) -> tuple[bool, ErrorCode, str]:
        if not self._gripper.ready():
            if self._allow_gripper_skip():
                return True, ErrorCode.INTERNAL_ERROR, ""
            return False, ErrorCode.INTERNAL_ERROR, "夹爪 action 未就绪"
        return self._gripper.close()

    def _publish_feedback(self, goal_handle, phase_prefix: str, progress: float) -> None:
        fb = PickObject.Feedback() if phase_prefix.startswith("pick") else PlaceAt.Feedback()
        fb.phase = phase_prefix
        fb.progress = float(max(0.0, min(1.0, progress)))
        goal_handle.publish_feedback(fb)

    def _approach_pose(self, target: PoseStamped, height_offset: float) -> tuple[bool, ErrorCode, str]:
        x = target.pose.position.x
        y = target.pose.position.y
        z = target.pose.position.z + height_offset
        return self._moveit.move_to_xyz(x, y, z)

    def _visual_servo_to_pose(
        self,
        target_fn: Callable[[], tuple[float, float, float] | None],
        goal_handle,
        phase_prefix: str,
    ) -> tuple[bool, ErrorCode, str]:
        if not self._servo.ensure_started():
            return False, ErrorCode.MOTION_PLANNING_FAILED, "moveit_servo 未就绪，请先 launch moveit_servo.launch.py"

        rate_hz = self.get_parameter("servo_rate_hz").value
        kp = self.get_parameter("kp").value
        thresh = self.get_parameter("converge_thresh_m").value
        max_iter = int(self.get_parameter("max_iter").value)
        period = 1.0 / rate_hz

        try:
            for i in range(max_iter):
                if self._check_abort() or goal_handle.is_cancel_requested:
                    self._servo.stop()
                    return False, ErrorCode.SERVO_ABORTED, "被中断"

                target_xyz = target_fn()
                if target_xyz is None:
                    self._servo.stop()
                    return False, ErrorCode.OBJECT_NOT_VISIBLE, "伺服目标不可用"

                ee = self._ee_position()
                if ee is None:
                    self._servo.stop()
                    return False, ErrorCode.INTERNAL_ERROR, "无法读取末端 TF"

                err = (target_xyz[0] - ee[0], target_xyz[1] - ee[1], target_xyz[2] - ee[2])
                dist = math.sqrt(err[0] ** 2 + err[1] ** 2 + err[2] ** 2)

                self._publish_feedback(goal_handle, f"{phase_prefix}_servo", i / max_iter)

                if dist < thresh:
                    self._servo.stop()
                    return True, ErrorCode.INTERNAL_ERROR, ""

                self._servo.apply_p_control(err[0], err[1], err[2], kp)
                time.sleep(period)

            self._servo.stop()
            return False, ErrorCode.SERVO_TIMEOUT, f"伺服 {max_iter} 次迭代未收敛"
        finally:
            self._servo.publish_twist(0.0, 0.0, 0.0)

    def _visual_servo_track_object(
        self, object_id: str, goal_handle, phase_prefix: str,
    ) -> tuple[bool, ErrorCode, str]:
        def target_fn() -> tuple[float, float, float] | None:
            pose = self._lookup(object_id)
            if pose is None:
                return None
            return (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

        return self._visual_servo_to_pose(target_fn, goal_handle, phase_prefix)

    def _lift_ee(self, delta_z: float) -> tuple[bool, ErrorCode, str]:
        ee = self._ee_position()
        if ee is None:
            return False, ErrorCode.INTERNAL_ERROR, "抬升前无法读取末端位置"
        return self._moveit.move_to_xyz(ee[0], ee[1], ee[2] + delta_z)

    def _verify_held(self, object_id: str, table_z_hint: float) -> tuple[bool, ErrorCode, str]:
        """Grasp OK if object no longer appears at table height (DESIGN §5.1)."""
        pose = self._lookup(object_id)
        if pose is None:
            return True, ErrorCode.INTERNAL_ERROR, ""
        if pose.pose.position.z < table_z_hint + 0.03:
            return False, ErrorCode.GRIPPER_SLIPPED, f"{object_id} 仍在桌面高度，疑似滑落"
        return True, ErrorCode.INTERNAL_ERROR, ""

    def _execute_pick(self, goal_handle):
        self._abort.clear()
        object_id = goal_handle.request.object_id
        self._current_task = f"pick_object({object_id})"

        target = self._lookup(object_id)
        if target is None:
            if self._scene and any(o.id == object_id for o in self._scene.objects):
                result = self._make_error_result(PickObject.Result(), ErrorCode.OBJECT_NOT_VISIBLE, f"{object_id} 已过期")
            else:
                result = self._make_error_result(PickObject.Result(), ErrorCode.UNKNOWN_OBJECT_ID, f"blackboard 无 {object_id}")
            goal_handle.abort()
            self._current_task = ""
            return result

        table_z = target.pose.position.z
        x, y, z = target.pose.position.x, target.pose.position.y, target.pose.position.z
        if DEFAULT_ENVELOPE.check_or_error(x, y, z):
            result = self._make_error_result(PickObject.Result(), ErrorCode.OUT_OF_REACH, f"{object_id} 超出工作空间")
            goal_handle.abort()
            self._current_task = ""
            return result

        ok, code, reason = self._ensure_gripper()
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        self._publish_feedback(goal_handle, "pick_open_gripper", 0.05)
        ok, code, reason = self._gripper_open()
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        self._publish_feedback(goal_handle, "pick_approach", 0.15)
        ok, code, reason = self._approach_pose(target, self.get_parameter("approach_height_m").value)
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        ok, code, reason = self._visual_servo_track_object(object_id, goal_handle, "pick")
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        self._publish_feedback(goal_handle, "pick_grasp", 0.7)
        ok, code, reason = self._gripper_close()
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        self._publish_feedback(goal_handle, "pick_lift", 0.85)
        ok, code, reason = self._lift_ee(self.get_parameter("lift_height_m").value)
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        if self._gripper.ready():
            ok, code, reason = self._verify_held(object_id, table_z)
            if not ok:
                result = self._make_error_result(PickObject.Result(), code, reason)
                goal_handle.abort()
                self._current_task = ""
                return result

        self._held_object_id = object_id
        result = PickObject.Result()
        result.success = True
        goal_handle.succeed()
        self._current_task = ""
        return result

    def _execute_place(self, goal_handle):
        self._abort.clear()
        target_id = goal_handle.request.target_id
        offset = goal_handle.request.offset
        self._current_task = f"place_at({target_id}, {offset})"

        if offset not in VALID_OFFSETS:
            result = self._make_error_result(PlaceAt.Result(), ErrorCode.INTERNAL_ERROR, f"不支持 offset: {offset}")
            goal_handle.abort()
            self._current_task = ""
            return result

        base = self._lookup(target_id)
        if base is None:
            if self._scene and any(o.id == target_id for o in self._scene.objects):
                result = self._make_error_result(PlaceAt.Result(), ErrorCode.OBJECT_NOT_VISIBLE, f"{target_id} 已过期")
            else:
                result = self._make_error_result(PlaceAt.Result(), ErrorCode.UNKNOWN_OBJECT_ID, f"blackboard 无 {target_id}")
            goal_handle.abort()
            self._current_task = ""
            return result

        try:
            place_pose = resolve_offset(base, offset)
        except ValueError as exc:
            result = self._make_error_result(PlaceAt.Result(), ErrorCode.INTERNAL_ERROR, str(exc))
            goal_handle.abort()
            self._current_task = ""
            return result

        px, py, pz = place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z
        if DEFAULT_ENVELOPE.check_or_error(px, py, pz):
            result = self._make_error_result(PlaceAt.Result(), ErrorCode.OUT_OF_REACH, "放置点超出工作空间")
            goal_handle.abort()
            self._current_task = ""
            return result

        self._publish_feedback(goal_handle, "place_approach", 0.2)
        ok, code, reason = self._approach_pose(place_pose, self.get_parameter("approach_height_m").value)
        if not ok:
            result = self._make_error_result(PlaceAt.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        place_xyz = (px, py, pz)

        def place_target_fn() -> tuple[float, float, float] | None:
            return place_xyz

        ok, code, reason = self._visual_servo_to_pose(place_target_fn, goal_handle, "place")
        if not ok:
            result = self._make_error_result(PlaceAt.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        self._publish_feedback(goal_handle, "place_release", 0.8)
        ok, code, reason = self._gripper_open()
        if not ok:
            result = self._make_error_result(PlaceAt.Result(), code, reason)
            goal_handle.abort()
            self._current_task = ""
            return result

        self._held_object_id = None
        result = PlaceAt.Result()
        result.success = True
        goal_handle.succeed()
        self._current_task = ""
        return result


def main() -> None:
    rclpy.init()
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
