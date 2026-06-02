"""moveit_servo Cartesian velocity client (DESIGN.md §5.1, §5.4)."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServoHelper:
    """Publish delta twists to moveit_servo and manage start/stop lifecycle.

    Humble moveit_msgs has no ServoCommandType; twist input is selected via
    moveit_servo yaml (cartesian_command_in_topic).
    """

    def __init__(
        self,
        node: Node,
        *,
        servo_node_name: str = "servo_node",
        twist_topic: str = "/servo_node/delta_twist_cmds",
        planning_frame: str = "panda_link0",
    ) -> None:
        self._node = node
        self._planning_frame = planning_frame
        self._started = False
        self._twist_pub = node.create_publisher(TwistStamped, twist_topic, 10)
        self._start_client = node.create_client(Trigger, f"/{servo_node_name}/start_servo")
        self._stop_client = node.create_client(Trigger, f"/{servo_node_name}/stop_servo")

    def ensure_started(self, timeout_sec: float = 3.0) -> bool:
        if self._started:
            return True
        if not self._call_trigger(self._start_client, timeout_sec):
            self._node.get_logger().error("moveit_servo start_servo 失败")
            return False
        self._started = True
        return True

    def stop(self) -> None:
        self.publish_twist(0.0, 0.0, 0.0)
        if self._started:
            self._call_trigger(self._stop_client, 1.0)
            self._started = False

    def publish_twist(self, vx: float, vy: float, vz: float) -> None:
        msg = TwistStamped()
        msg.header.frame_id = self._planning_frame
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self._twist_pub.publish(msg)

    def apply_p_control(self, err_x: float, err_y: float, err_z: float, kp: float) -> None:
        self.publish_twist(err_x * kp, err_y * kp, err_z * kp)

    def _call_trigger(self, client, timeout_sec: float) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False
        future = client.call_async(Trigger.Request())
        self._spin_until_done(future, timeout_sec)
        if not future.done() or future.result() is None:
            return False
        return bool(future.result().success)

    def _spin_until_done(self, future, timeout_sec: float) -> None:
        deadline = self._node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and not future.done():
            if self._node.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(self._node, timeout_sec=0.02)
