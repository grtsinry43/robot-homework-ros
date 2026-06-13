"""Pick/place inner loop — visual servo + gripper (DESIGN.md §5.1)."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable

import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, TransformStamped
from pick_place_msgs.action import PickObject, PlaceAt
from pick_place_msgs.srv import AbortTask
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scene_state_msgs.msg import SceneState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .errors import ErrorCode, DEFAULT_SUGGESTIONS
from .gripper_helper import GripperHelper
from .moveit_helper import MoveItHelper
from .offset_resolver import VALID_OFFSETS, resolve_offset
from .planning_scene_helper import PlanningSceneHelper
from .servo_helper import ServoHelper
from .workspace_envelope import DEFAULT_ENVELOPE

import tf2_geometry_msgs  # noqa: F401


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
        self.declare_parameter("pre_grasp_height_m", 0.05)
        self.declare_parameter("ee_grasp_offset_m", 0.103)
        self.declare_parameter("skip_servo_within_m", 0.12)
        self.declare_parameter("skip_place_servo", True)
        self.declare_parameter("skip_all_servo", False)
        self.declare_parameter("servo_max_duration_sec", 20.0)
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
        self.declare_parameter("hand_frame", "panda_hand")
        self.declare_parameter("use_planning_scene_attach", True)

        self._cb_group = ReentrantCallbackGroup()
        self._abort = threading.Event()
        self._current_task = ""
        self._scene: SceneState | None = None
        self._held_object_id: str | None = None
        self._gazebo_attached_id: str | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        planning_frame = self.get_parameter("planning_frame").value
        self._moveit = MoveItHelper(
            self,
            group_name=self.get_parameter("move_group").value,
            ee_link=self.get_parameter("ee_link").value,
            planning_frame=planning_frame,
            tf_buffer=self._tf_buffer,
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
        self._planning_scene = PlanningSceneHelper(
            self,
            planning_frame=planning_frame,
            attach_link=self.get_parameter("hand_frame").value,
        )

        self.create_subscription(SceneState, "/scene_state", self._on_scene, 10)
        self._grasp_hint_pub = self.create_publisher(String, "/gazebo_gripper/grasp_target", 10)
        self.create_subscription(
            String, "/gazebo_gripper/attached_object", self._on_gazebo_attached, 10,
        )
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

    def _on_gazebo_attached(self, msg: String) -> None:
        self._gazebo_attached_id = msg.data.strip() or None

    def _label_for(self, object_id: str) -> str:
        if self._scene is None:
            return "red_block"
        for obj in self._scene.objects:
            if obj.id == object_id:
                return obj.label
        if "plate" in object_id:
            return "blue_plate"
        if "green" in object_id:
            return "green_block"
        return "red_block"

    def _use_planning_scene(self) -> bool:
        return bool(self.get_parameter("use_planning_scene_attach").value)

    def _object_pose_in_hand(self, object_id: str) -> Pose | None:
        target = self._lookup(object_id)
        if target is None:
            return None
        hand_frame = self.get_parameter("hand_frame").value
        planning_frame = self.get_parameter("planning_frame").value
        try:
            hand_tf = self._tf_buffer.lookup_transform(
                hand_frame,
                planning_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warning(f"hand TF for attach failed: {exc}")
            return None

        obj_pt = PointStamped()
        obj_pt.header.frame_id = planning_frame
        obj_pt.header.stamp = self.get_clock().now().to_msg()
        obj_pt.point = target.pose.position
        obj_in_hand = tf2_geometry_msgs.do_transform_point(obj_pt, hand_tf)
        pose = Pose()
        pose.position = obj_in_hand.point
        pose.orientation.w = 1.0
        return pose

    def _sync_planning_world(self, *, exclude_ids: tuple[str, ...] = ()) -> None:
        if not self._use_planning_scene() or not self._planning_scene.ready():
            return
        if not self._planning_scene.sync_world_from_scene(self._scene, exclude_ids=exclude_ids):
            self.get_logger().warning("planning scene world sync failed")

    def _remove_planning_object(self, object_id: str) -> None:
        if not self._use_planning_scene() or not self._planning_scene.ready():
            return
        if not self._planning_scene.remove_world_object(object_id):
            self.get_logger().warning(f"planning scene remove failed for {object_id}")

    def _attach_planning_object(self, object_id: str) -> bool:
        if not self._use_planning_scene() or not self._planning_scene.ready():
            return True
        pose_in_hand = self._object_pose_in_hand(object_id)
        if pose_in_hand is None:
            self.get_logger().warning(f"skip planning attach: no pose for {object_id}")
            return False
        label = self._label_for(object_id)
        ok = self._planning_scene.attach_to_hand(object_id, label, pose_in_hand)
        if not ok:
            self.get_logger().warning(f"planning scene attach failed for {object_id}")
        return ok

    def _detach_planning_object(self, object_id: str, world_pose: Pose) -> None:
        if not self._use_planning_scene() or not self._planning_scene.ready():
            return
        label = self._label_for(object_id)
        if not self._planning_scene.detach_to_world(object_id, label, world_pose):
            self.get_logger().warning(f"planning scene detach failed for {object_id}")

    def _verify_real_grasp(self, object_id: str, ee_z_before_lift: float) -> tuple[bool, ErrorCode, str]:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self._gazebo_attached_id == object_id:
                break
            time.sleep(0.05)
        else:
            return False, ErrorCode.GRIPPER_SLIPPED, f"Gazebo 未 attach {object_id}"

        ee = self._ee_position()
        min_lift = float(self.get_parameter("lift_height_m").value) * 0.35
        if ee is None or ee[2] < ee_z_before_lift + min_lift:
            return False, ErrorCode.GRIPPER_SLIPPED, f"{object_id} 抬升不足，疑似未抓牢"
        return True, ErrorCode.INTERNAL_ERROR, ""

    def _describe_goal(self, goal) -> str:
        if hasattr(goal, "object_id"):
            return f"pick_object(object_id={goal.object_id})"
        if hasattr(goal, "target_id") and hasattr(goal, "offset"):
            return f"place_at(target_id={goal.target_id}, offset={goal.offset})"
        return type(goal).__name__

    def _goal_ok(self, goal) -> GoalResponse:
        desc = self._describe_goal(goal)
        if self._current_task:
            self.get_logger().warning(
                f"goal rejected: {desc}; current_task={self._current_task}; held={self._held_object_id}",
            )
            return GoalResponse.REJECT
        self.get_logger().info(f"goal accepted: {desc}; held={self._held_object_id}")
        return GoalResponse.ACCEPT

    def _cancel_ok(self, _goal) -> CancelResponse:
        self.get_logger().warning(f"cancel requested: current_task={self._current_task}; held={self._held_object_id}")
        self._abort.set()
        self._servo.stop()
        return CancelResponse.ACCEPT

    def _handle_abort(self, _request: AbortTask.Request, response: AbortTask.Response) -> AbortTask.Response:
        what = self._current_task
        self.get_logger().warning(f"abort requested: current_task={what or 'idle'}; held={self._held_object_id}")
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

    def _link8_grasp_z(self, object_z: float) -> float:
        """Flange height so finger tips reach object center (link8 is ~10.3cm above fingers)."""
        return object_z + float(self.get_parameter("ee_grasp_offset_m").value)

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

    def _approach_place_pose(self, target: PoseStamped, goal_handle) -> tuple[bool, ErrorCode, str]:
        """Move to the place approach point via clearance waypoints."""
        x = target.pose.position.x
        y = target.pose.position.y
        link8_z = self._link8_grasp_z(target.pose.position.z)
        approach_z = link8_z + float(self.get_parameter("approach_height_m").value)

        ee = self._ee_position()
        if ee is None:
            self.get_logger().warning("place approach: EE TF unavailable, falling back to direct approach")
            return self._moveit.move_to_xyz(x, y, approach_z)

        clearance_z = max(ee[2], approach_z)
        if clearance_z - ee[2] > 0.015:
            self._publish_feedback(goal_handle, "place_clearance_lift", 0.12)
            self.get_logger().info(
                f"place clearance lift: xyz=({ee[0]:.3f}, {ee[1]:.3f}, {clearance_z:.3f})",
            )
            ok, code, reason = self._moveit.move_to_xyz(ee[0], ee[1], clearance_z)
            if not ok:
                return ok, code, reason

        # Lateral move over the plate at clearance height: free-space, keep OMPL joint-space.
        self._publish_feedback(goal_handle, "place_clearance_translate", 0.16)
        self.get_logger().info(f"place clearance translate: xyz=({x:.3f}, {y:.3f}, {clearance_z:.3f})")
        ok, code, reason = self._moveit.move_to_xyz(x, y, clearance_z)
        if not ok:
            return ok, code, reason

        if abs(clearance_z - approach_z) > 0.015:
            self._publish_feedback(goal_handle, "place_approach_descend", 0.2)
            self.get_logger().info(f"place approach descend: xyz=({x:.3f}, {y:.3f}, {approach_z:.3f})")
            ok, code, reason = self._moveit.move_to_xyz(x, y, approach_z)
            if not ok:
                return ok, code, reason

        return True, ErrorCode.INTERNAL_ERROR, ""

    def _maybe_servo_to_xyz(
        self,
        target_xyz: tuple[float, float, float],
        goal_handle,
        phase_prefix: str,
        *,
        fallback_track: Callable[[], tuple[bool, ErrorCode, str]] | None = None,
    ) -> tuple[bool, ErrorCode, str]:
        if bool(self.get_parameter("skip_all_servo").value):
            self.get_logger().info(f"{phase_prefix}: skip_all_servo — 仅 MoveIt")
            return self._moveit.move_to_xyz(target_xyz[0], target_xyz[1], target_xyz[2])

        skip_dist = float(self.get_parameter("skip_servo_within_m").value)
        ee = self._ee_position()
        if ee is not None and math.dist(ee, target_xyz) < skip_dist:
            self.get_logger().info(
                f"{phase_prefix}: skip servo (EE 距目标 {math.dist(ee, target_xyz):.3f}m < {skip_dist}m)",
            )
            return True, ErrorCode.INTERNAL_ERROR, ""

        if fallback_track is not None:
            return fallback_track()
        return self._visual_servo_to_pose(lambda: target_xyz, goal_handle, phase_prefix)

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
        max_duration = float(self.get_parameter("servo_max_duration_sec").value)
        period = 1.0 / rate_hz
        t0 = time.monotonic()

        try:
            for i in range(max_iter):
                if time.monotonic() - t0 > max_duration:
                    self.get_logger().warning(f"{phase_prefix}_servo 超时 {max_duration}s，停止伺服")
                    self._servo.stop()
                    return True, ErrorCode.INTERNAL_ERROR, ""
                if self._abort.is_set() or goal_handle.is_cancel_requested:
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
        self,
        object_id: str,
        goal_handle,
        phase_prefix: str,
        fallback_xyz: tuple[float, float, float] | None = None,
    ) -> tuple[bool, ErrorCode, str]:
        """Track object pose; keep last known (or fallback) when perception briefly drops."""
        last_xyz = fallback_xyz

        def target_fn() -> tuple[float, float, float] | None:
            nonlocal last_xyz
            pose = self._lookup(object_id)
            if pose is not None:
                last_xyz = (
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                )
            return last_xyz

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
        self.get_logger().info(
            f"pick execute start: object_id={object_id}; current_task_before={self._current_task or 'idle'}; "
            f"held={self._held_object_id}",
        )
        self._current_task = f"pick_object({object_id})"
        started_at = time.monotonic()
        try:
            result = self._execute_pick_body(goal_handle, object_id)
            self.get_logger().info(
                f"pick execute result: object_id={object_id}; success={result.success}; "
                f"code={result.code or 'none'}; elapsed={time.monotonic() - started_at:.1f}s; "
                f"held={self._held_object_id}",
            )
            return result
        finally:
            self._servo.stop()
            self.get_logger().info(
                f"pick execute cleanup: clearing current_task={self._current_task or 'idle'}; "
                f"held={self._held_object_id}",
            )
            self._current_task = ""

    def _execute_pick_body(self, goal_handle, object_id: str):
        target = self._lookup(object_id)
        if target is None:
            if self._scene and any(o.id == object_id for o in self._scene.objects):
                result = self._make_error_result(PickObject.Result(), ErrorCode.OBJECT_NOT_VISIBLE, f"{object_id} 已过期")
            else:
                result = self._make_error_result(PickObject.Result(), ErrorCode.UNKNOWN_OBJECT_ID, f"blackboard 无 {object_id}")
            goal_handle.abort()
            return result

        table_z = target.pose.position.z
        x, y, z = target.pose.position.x, target.pose.position.y, target.pose.position.z
        link8_grasp_z = self._link8_grasp_z(z)
        approach_z = link8_grasp_z + float(self.get_parameter("approach_height_m").value)
        if DEFAULT_ENVELOPE.check_or_error(x, y, approach_z):
            result = self._make_error_result(PickObject.Result(), ErrorCode.OUT_OF_REACH, f"{object_id} 超出工作空间")
            goal_handle.abort()
            return result

        # Include the target block as a collision object so MoveIt routes the approach
        # AROUND it instead of sweeping the hand sideways through it (which knocked the
        # block off the table). It's removed from collision just before the final vertical
        # descent below, so the grasp move itself doesn't collide with it.
        self._sync_planning_world()

        ok, code, reason = self._ensure_gripper()
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "pick_open_gripper", 0.05)
        ok, code, reason = self._gripper_open()
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "pick_approach", 0.15)
        ok, code, reason = self._moveit.move_to_xyz(x, y, approach_z)
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            return result

        pre_z = link8_grasp_z + float(self.get_parameter("pre_grasp_height_m").value)
        self._publish_feedback(goal_handle, "pick_pre_grasp", 0.22)
        ok, code, reason = self._moveit.move_to_xyz(x, y, pre_z)
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            return result

        # Hand is now directly above the block. Drop the block from the collision world so
        # the final vertical descent into the grasp pose doesn't collide with it.
        self._remove_planning_object(object_id)

        grasp_z = link8_grasp_z
        if not self._gripper.ready() and self._allow_gripper_skip():
            grasp_z = link8_grasp_z + min(float(self.get_parameter("pre_grasp_height_m").value), 0.03)
            self.get_logger().info(
                f"pick: gripper skipped, using safe simulated grasp height z={grasp_z:.3f}",
            )

        grasp_xyz = (x, y, grasp_z)
        ok, code, reason = self._maybe_servo_to_xyz(
            grasp_xyz, goal_handle, "pick",
            fallback_track=lambda: self._visual_servo_track_object(
                object_id, goal_handle, "pick", fallback_xyz=grasp_xyz,
            ),
        )
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "pick_grasp", 0.7)
        self._grasp_hint_pub.publish(String(data=object_id))
        ok, code, reason = self._gripper_close()
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            return result

        # Planning-scene attach is collision bookkeeping for carry-time avoidance; the
        # physical DetachableJoint already holds the object, so a failure here must not
        # fail the pick.
        if not self._attach_planning_object(object_id):
            self.get_logger().warning(
                f"planning scene attach failed for {object_id}; physical grasp still holds, continuing",
            )

        ee_before_lift = self._ee_position()
        ee_z_before = ee_before_lift[2] if ee_before_lift else 0.0

        self._publish_feedback(goal_handle, "pick_lift", 0.85)
        ok, code, reason = self._lift_ee(self.get_parameter("lift_height_m").value)
        if not ok:
            result = self._make_error_result(PickObject.Result(), code, reason)
            goal_handle.abort()
            return result

        if self._gripper.ready() and self._allow_gripper_skip():
            ok, code, reason = self._verify_held(object_id, table_z)
            if not ok:
                result = self._make_error_result(PickObject.Result(), code, reason)
                goal_handle.abort()
                self._current_task = ""
                return result
        elif self._gripper.ready() and not self._allow_gripper_skip():
            ok, code, reason = self._verify_real_grasp(object_id, ee_z_before)
            if not ok:
                result = self._make_error_result(PickObject.Result(), code, reason)
                goal_handle.abort()
                return result

        self._held_object_id = object_id
        result = PickObject.Result()
        result.success = True
        goal_handle.succeed()
        return result

    def _execute_place(self, goal_handle):
        self._abort.clear()
        target_id = goal_handle.request.target_id
        offset = goal_handle.request.offset
        self.get_logger().info(
            f"place execute start: target_id={target_id}; offset={offset}; "
            f"current_task_before={self._current_task or 'idle'}; held={self._held_object_id}",
        )
        self._current_task = f"place_at({target_id}, {offset})"
        started_at = time.monotonic()
        try:
            result = self._execute_place_body(goal_handle, target_id, offset)
            self.get_logger().info(
                f"place execute result: target_id={target_id}; offset={offset}; success={result.success}; "
                f"code={result.code or 'none'}; elapsed={time.monotonic() - started_at:.1f}s; "
                f"held={self._held_object_id}",
            )
            return result
        finally:
            self._servo.stop()
            self.get_logger().info(
                f"place execute cleanup: clearing current_task={self._current_task or 'idle'}; "
                f"held={self._held_object_id}",
            )
            self._current_task = ""

    def _execute_place_body(self, goal_handle, target_id: str, offset: str):
        self.get_logger().info(f"place body start: target_id={target_id}; offset={offset}; held={self._held_object_id}")
        if offset not in VALID_OFFSETS:
            self.get_logger().warning(f"place rejected: unsupported offset={offset}")
            result = self._make_error_result(PlaceAt.Result(), ErrorCode.INTERNAL_ERROR, f"不支持 offset: {offset}")
            goal_handle.abort()
            return result

        base = self._lookup(target_id)
        if base is None:
            scene_ids = [obj.id for obj in self._scene.objects] if self._scene else []
            self.get_logger().warning(f"place rejected: target_id={target_id} unavailable; scene_ids={scene_ids}")
            if self._scene and any(o.id == target_id for o in self._scene.objects):
                result = self._make_error_result(PlaceAt.Result(), ErrorCode.OBJECT_NOT_VISIBLE, f"{target_id} 已过期")
            else:
                result = self._make_error_result(PlaceAt.Result(), ErrorCode.UNKNOWN_OBJECT_ID, f"blackboard 无 {target_id}")
            goal_handle.abort()
            return result

        try:
            place_pose = resolve_offset(base, offset)
        except ValueError as exc:
            self.get_logger().warning(f"place rejected: resolve_offset failed for {target_id}/{offset}: {exc}")
            result = self._make_error_result(PlaceAt.Result(), ErrorCode.INTERNAL_ERROR, str(exc))
            goal_handle.abort()
            return result

        px, py, pz = place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z
        place_link8_z = self._link8_grasp_z(pz)
        self.get_logger().info(
            f"place resolved: target_id={target_id}; offset={offset}; "
            f"xyz=({px:.3f}, {py:.3f}, {pz:.3f}); link8_z={place_link8_z:.3f}",
        )
        if DEFAULT_ENVELOPE.check_or_error(px, py, place_link8_z):
            self.get_logger().warning(
                f"place rejected: resolved point out of workspace xyz=({px:.3f}, {py:.3f}, {pz:.3f})",
            )
            result = self._make_error_result(PlaceAt.Result(), ErrorCode.OUT_OF_REACH, "放置点超出工作空间")
            goal_handle.abort()
            return result

        self._sync_planning_world(exclude_ids=(self._held_object_id or "",))

        self._publish_feedback(goal_handle, "place_approach", 0.2)
        self.get_logger().info(f"place approach: target_id={target_id}; offset={offset}")
        ok, code, reason = self._approach_place_pose(place_pose, goal_handle)
        if not ok:
            self.get_logger().warning(f"place approach failed: code={code.value}; reason={reason}")
            result = self._make_error_result(PlaceAt.Result(), code, reason)
            goal_handle.abort()
            return result

        place_xyz = (px, py, place_link8_z)
        if bool(self.get_parameter("skip_place_servo").value) or bool(
            self.get_parameter("skip_all_servo").value
        ):
            self.get_logger().info("place: MoveIt 直达放置点（无伺服）")
            # Final drop to the release point uses OMPL's sphere tolerance + collision
            # avoidance: the release pose sits low among plate+table+held-block geometry,
            # so it needs that slack to find a valid pose.
            ok, code, reason = self._moveit.move_to_xyz(px, py, place_link8_z)
        else:
            ok, code, reason = self._maybe_servo_to_xyz(place_xyz, goal_handle, "place")
        if not ok:
            self.get_logger().warning(f"place move failed: code={code.value}; reason={reason}")
            result = self._make_error_result(PlaceAt.Result(), code, reason)
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "place_release", 0.8)
        self.get_logger().info(f"place release: target_id={target_id}; offset={offset}")
        ok, code, reason = self._gripper_open()
        if not ok:
            self.get_logger().warning(f"place release failed: code={code.value}; reason={reason}")
            result = self._make_error_result(PlaceAt.Result(), code, reason)
            goal_handle.abort()
            return result

        if self._held_object_id:
            self._detach_planning_object(self._held_object_id, place_pose.pose)

        self._held_object_id = None
        result = PlaceAt.Result()
        result.success = True
        goal_handle.succeed()
        return result


def main() -> None:
    rclpy.init()
    node = ExecutorNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
