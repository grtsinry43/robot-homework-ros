#!/usr/bin/env python3
"""Merge Gazebo arm /joint_states_hw with static gripper joints for MoveIt."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class GripperJointStateMerger(Node):
    def __init__(self) -> None:
        super().__init__("gripper_joint_state_merger")
        self.declare_parameter("input_topic", "/joint_states_hw")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("finger_open_pos", 0.035)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._finger_open = float(self.get_parameter("finger_open_pos").value)
        self._last_arm: JointState | None = None

        self._pub = self.create_publisher(JointState, output_topic, qos_profile_sensor_data)
        self.create_subscription(
            JointState, input_topic, self._on_arm_state, qos_profile_sensor_data,
        )
        self.create_timer(0.01, self._publish_merged)
        self.get_logger().info(
            f"merging {input_topic} + finger joints -> {output_topic}",
        )

    def _merge(self, msg: JointState) -> JointState:
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity) if msg.velocity else []
        out.effort = list(msg.effort) if msg.effort else []

        finger1_pos = self._finger_open
        if "panda_finger_joint1" in out.name:
            idx = out.name.index("panda_finger_joint1")
            if idx < len(out.position):
                finger1_pos = out.position[idx]

        for finger in ("panda_finger_joint1", "panda_finger_joint2"):
            if finger in out.name:
                continue
            out.name.append(finger)
            out.position.append(finger1_pos)
            if out.velocity:
                out.velocity.append(0.0)
            if out.effort:
                out.effort.append(0.0)
        return out

    def _on_arm_state(self, msg: JointState) -> None:
        self._last_arm = msg
        self._pub.publish(self._merge(msg))

    def _publish_merged(self) -> None:
        if self._last_arm is None:
            return
        self._pub.publish(self._merge(self._last_arm))


def main() -> None:
    rclpy.init()
    node = GripperJointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
