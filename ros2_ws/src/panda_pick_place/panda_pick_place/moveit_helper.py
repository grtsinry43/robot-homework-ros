"""MoveIt2 MoveGroup helper for macro motions (executor fallback before moveit_servo)."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .errors import ErrorCode
from .workspace_envelope import DEFAULT_ENVELOPE


class MoveItHelper:
    """Send position goals to move_action (panda_link8 sphere constraint)."""

    def __init__(self, node: Node, group_name: str = "panda_arm", ee_link: str = "panda_link8") -> None:
        self._node = node
        self._group_name = group_name
        self._ee_link = ee_link
        self._client = ActionClient(node, MoveGroup, "move_action")
        self._viz_pub = node.create_publisher(PoseStamped, "/llm_target_pose", 10)

    def server_ready(self) -> bool:
        return self._client.server_is_ready()

    def move_to_xyz(self, x: float, y: float, z: float, timeout_sec: float = 30.0) -> tuple[bool, ErrorCode, str]:
        if DEFAULT_ENVELOPE.check_or_error(x, y, z):
            return False, ErrorCode.OUT_OF_REACH, "目标超出安全工作空间"

        if not self.server_ready():
            return False, ErrorCode.MOTION_PLANNING_FAILED, "MoveIt2 move_action 未就绪"

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        viz = PoseStamped()
        viz.header.frame_id = "panda_link0"
        viz.header.stamp = self._node.get_clock().now().to_msg()
        viz.pose = pose
        self._viz_pub.publish(viz)

        goal = MoveGroup.Goal()
        goal.request.group_name = self._group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.4
        goal.request.max_acceleration_scaling_factor = 0.4

        constraint = PositionConstraint()
        constraint.header.frame_id = "panda_link0"
        constraint.link_name = self._ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        constraint.constraint_region.primitives = [sphere]
        constraint.constraint_region.primitive_poses = [pose]
        constraint.weight = 1.0
        goal.request.goal_constraints = [Constraints(position_constraints=[constraint])]

        send_future = self._client.send_goal_async(goal)
        self._spin_until_done(send_future, min(timeout_sec, 5.0))
        if not send_future.done():
            return False, ErrorCode.MOTION_PLANNING_FAILED, "提交 MoveIt 目标超时"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, ErrorCode.MOTION_PLANNING_FAILED, "MoveIt 拒绝目标"

        result_future = goal_handle.get_result_async()
        self._spin_until_done(result_future, timeout_sec)
        if not result_future.done():
            return False, ErrorCode.SERVO_TIMEOUT, "MoveIt 执行超时"

        wrapped = result_future.result()
        if wrapped is None:
            return False, ErrorCode.INTERNAL_ERROR, "MoveIt 无结果"

        error_code = wrapped.result.error_code.val if wrapped.result.error_code else -1
        if error_code == 1:
            return True, ErrorCode.INTERNAL_ERROR, ""

        return False, ErrorCode.MOTION_PLANNING_FAILED, f"MoveIt error_code={error_code}"

    def _spin_until_done(self, future, timeout_sec: float) -> None:
        deadline = self._node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and not future.done():
            if self._node.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(self._node, timeout_sec=0.05)
