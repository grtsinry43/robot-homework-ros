"""Franka Hand gripper via franka_gripper actions (DESIGN.md §8 D4)."""

from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .errors import ErrorCode


class GripperHelper:
    def __init__(
        self,
        node: Node,
        *,
        move_action: str = "/franka_gripper/move",
        grasp_action: str = "/franka_gripper/grasp",
        open_width_m: float = 0.08,
        grasp_width_m: float = 0.035,
        grasp_speed: float = 0.1,
        grasp_force: float = 5.0,
    ) -> None:
        self._node = node
        self._open_width = open_width_m
        self._grasp_width = grasp_width_m
        self._grasp_speed = grasp_speed
        self._grasp_force = grasp_force
        self._move_action = move_action
        self._grasp_action = grasp_action
        self._move_client: ActionClient | None = None
        self._grasp_client: ActionClient | None = None
        self._init_clients()

    def _init_clients(self) -> None:
        try:
            from franka_msgs.action import Grasp, Move

            self._Move = Move
            self._Grasp = Grasp
            self._move_client = ActionClient(self._node, Move, self._move_action)
            self._grasp_client = ActionClient(self._node, Grasp, self._grasp_action)
        except ImportError:
            self._node.get_logger().warning(
                "franka_msgs 未安装 — 夹爪需 franka_ros2 vendor 依赖"
            )

    def ready(self) -> bool:
        return (
            self._move_client is not None
            and self._grasp_client is not None
            and self._move_client.server_is_ready()
            and self._grasp_client.server_is_ready()
        )

    def open(self, timeout_sec: float = 10.0) -> tuple[bool, ErrorCode, str]:
        return self._move(self._open_width, timeout_sec)

    def close(self, timeout_sec: float = 10.0) -> tuple[bool, ErrorCode, str]:
        if self._grasp_client is None:
            return False, ErrorCode.INTERNAL_ERROR, "Grasp action client 未初始化"
        if not self._grasp_client.server_is_ready():
            return False, ErrorCode.INTERNAL_ERROR, f"{self._grasp_action} 未就绪"

        goal = self._Grasp.Goal()
        goal.width = self._grasp_width
        goal.speed = self._grasp_speed
        goal.force = self._grasp_force
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005

        ok, reason = self._send_action(self._grasp_client, goal, timeout_sec)
        if not ok:
            return False, ErrorCode.GRASP_PLANNING_FAILED, reason
        return True, ErrorCode.INTERNAL_ERROR, ""

    def _move(self, width: float, timeout_sec: float) -> tuple[bool, ErrorCode, str]:
        if self._move_client is None:
            return False, ErrorCode.INTERNAL_ERROR, "Move action client 未初始化"
        if not self._move_client.server_is_ready():
            return False, ErrorCode.INTERNAL_ERROR, f"{self._move_action} 未就绪"

        goal = self._Move.Goal()
        goal.width = width
        goal.speed = self._grasp_speed

        ok, reason = self._send_action(self._move_client, goal, timeout_sec)
        if not ok:
            return False, ErrorCode.GRASP_PLANNING_FAILED, reason
        return True, ErrorCode.INTERNAL_ERROR, ""

    def _send_action(self, client: ActionClient, goal, timeout_sec: float) -> tuple[bool, str]:
        send_future = client.send_goal_async(goal)
        self._spin_until_done(send_future, 5.0)
        if not send_future.done():
            return False, "提交夹爪目标超时"
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, "夹爪目标被拒绝"

        result_future = goal_handle.get_result_async()
        self._spin_until_done(result_future, timeout_sec)
        if not result_future.done():
            return False, "夹爪执行超时"
        wrapped = result_future.result()
        if wrapped is None:
            return False, "夹爪无结果"
        if wrapped.status == 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            return True, ""
        return False, f"夹爪 action status={wrapped.status}"

    def _spin_until_done(self, future, timeout_sec: float) -> None:
        deadline = self._node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and not future.done():
            if self._node.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(self._node, timeout_sec=0.02)
