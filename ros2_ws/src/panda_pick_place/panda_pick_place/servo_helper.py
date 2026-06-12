"""moveit_servo Cartesian velocity client (DESIGN.md §5.1, §5.4)."""

from __future__ import annotations

import time

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
        if not self._call_trigger(self._start_client, timeout_sec + 2.0):
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
        # The node is already spun by the executor's MultiThreadedExecutor; spinning it
        # again here would error/deadlock. Poll the future instead (same pattern as the
        # gripper/moveit helpers).
        if not self._wait_future(future, timeout_sec):
            return False
        result = future.result()
        return result is not None and bool(result.success)

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()
