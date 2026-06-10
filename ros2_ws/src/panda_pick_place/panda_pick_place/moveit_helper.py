"""MoveIt2 MoveGroup helper for macro motions (executor fallback before moveit_servo)."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException

from .errors import ErrorCode
from .workspace_envelope import DEFAULT_ENVELOPE

# Panda ready-pose-like orientation (top-down over desk) when TF is unavailable.
_DEFAULT_EE_QUAT = (0.9238795, 0.0, 0.3826834, 0.0)  # xyzw, ~45° pitch in link0
_POST_EXECUTION_SETTLE_SEC = 0.35


class MoveItHelper:
    """Send position goals to move_action (panda_link8 sphere constraint)."""

    def __init__(
        self,
        node: Node,
        group_name: str = "panda_arm",
        ee_link: str = "panda_link8",
        planning_frame: str = "panda_link0",
        tf_buffer: Buffer | None = None,
    ) -> None:
        self._node = node
        self._group_name = group_name
        self._ee_link = ee_link
        self._planning_frame = planning_frame
        self._tf_buffer = tf_buffer
        self._cb_group = ReentrantCallbackGroup()
        self._client = ActionClient(
            node, MoveGroup, "move_action", callback_group=self._cb_group,
        )
        self._viz_pub = node.create_publisher(PoseStamped, "/llm_target_pose", 10)

    def server_ready(self) -> bool:
        return self._client.server_is_ready()

    def _goal_orientation(self) -> tuple[float, float, float, float]:
        if self._tf_buffer is not None:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._planning_frame,
                    self._ee_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
                q = tf.transform.rotation
                return (q.x, q.y, q.z, q.w)
            except TransformException:
                pass
        return _DEFAULT_EE_QUAT

    def move_to_xyz(self, x: float, y: float, z: float, timeout_sec: float = 30.0) -> tuple[bool, ErrorCode, str]:
        if DEFAULT_ENVELOPE.check_or_error(x, y, z):
            return False, ErrorCode.OUT_OF_REACH, "目标超出安全工作空间"

        if not self.server_ready():
            return False, ErrorCode.MOTION_PLANNING_FAILED, "MoveIt2 move_action 未就绪"

        qx, qy, qz, qw = self._goal_orientation()
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        viz = PoseStamped()
        viz.header.frame_id = self._planning_frame
        viz.header.stamp = self._node.get_clock().now().to_msg()
        viz.pose = pose
        self._viz_pub.publish(viz)

        goal = MoveGroup.Goal()
        goal.request.group_name = self._group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        constraint = PositionConstraint()
        constraint.header.frame_id = self._planning_frame
        constraint.link_name = self._ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]
        constraint.constraint_region.primitives = [sphere]
        constraint.constraint_region.primitive_poses = [pose]
        constraint.weight = 1.0
        goal.request.goal_constraints = [Constraints(position_constraints=[constraint])]

        send_future = self._client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec):
            return False, ErrorCode.MOTION_PLANNING_FAILED, "提交 MoveIt 目标超时"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, ErrorCode.MOTION_PLANNING_FAILED, "MoveIt 拒绝目标"

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec):
            return False, ErrorCode.SERVO_TIMEOUT, "MoveIt 执行超时"

        wrapped = result_future.result()
        if wrapped is None:
            return False, ErrorCode.INTERNAL_ERROR, "MoveIt 无结果"

        error_code = wrapped.result.error_code.val if wrapped.result.error_code else -1
        if error_code == 1:
            time.sleep(_POST_EXECUTION_SETTLE_SEC)
            return True, ErrorCode.INTERNAL_ERROR, ""

        hint = {
            -6: "TIMED_OUT",
            -4: "CONTROL_FAILED",
            -2: "INVALID_MOTION_PLAN",
            -1: "PLANNING_FAILED",
        }.get(error_code, "")
        suffix = f" ({hint})" if hint else ""
        return False, ErrorCode.MOTION_PLANNING_FAILED, f"MoveIt error_code={error_code}{suffix}"

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()
