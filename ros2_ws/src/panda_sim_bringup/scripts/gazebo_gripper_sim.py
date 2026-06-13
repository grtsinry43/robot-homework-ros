#!/usr/bin/env python3
"""Gazebo shim for franka_gripper Move/Grasp actions.

Grasping is a *physical* DetachableJoint attach (declared in panda_gz.urdf.xacro),
not a set_pose drag: on grasp we publish on /grasp/<model>/attach so gz-sim creates a
real fixed joint between panda_hand and the object; on open we publish .../detach.
This node only (a) drives the finger trajectory, (b) picks which object to grab, and
(c) flips the attach/detach topics. The physics engine carries the object after that.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Grasp, Move
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from scene_state_msgs.msg import SceneState
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FINGER_JOINT = "panda_finger_joint1"
MAX_FINGER_POS = 0.04


def width_to_joint(width_m: float) -> float:
    return max(0.0, min(MAX_FINGER_POS, width_m / 2.0))


def joint_to_width(joint_pos: float) -> float:
    return max(0.0, min(MAX_FINGER_POS * 2.0, joint_pos * 2.0))


class GazeboGripperSim(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_gripper_sim")
        self.declare_parameter("hand_frame", "panda_hand")
        self.declare_parameter("planning_frame", "panda_link0")
        self.declare_parameter("gripper_controller", "panda_gripper_controller")
        self.declare_parameter("graspable_models", ["red_block_01", "green_block_01"])
        # Horizontal alignment gate: hand must be roughly over the object. MoveIt's vertical
        # descent lands the flange a few cm off in xy run-to-run near the workspace; 0.05 m
        # rejected otherwise-valid grasps. snap_to_gripper re-centers the block on attach, so
        # a slightly off-center hand still grips correctly — 0.07 m absorbs the descent drift.
        self.declare_parameter("grasp_xy_tol_m", 0.07)
        # Vertical gate. At a correct top-down grasp panda_hand sits ~ee_grasp_offset
        # (0.103 m) above the object center (fingers hang ~10 cm below the hand origin).
        # The band rejects gross misses (a ~12 cm-high hand left the block dangling under
        # the arm without ever being gripped). The upper bound must still tolerate MoveIt's
        # run-to-run descent slop near the workspace edge: with the vertical-down constraint
        # the planner sometimes settles the flange ~3-4 cm high (a valid, xy-aligned grasp
        # attempt, just shy on z). snap_to_gripper re-centers the block on attach, so a
        # slightly high hand still grips correctly. Band: 0.083 .. 0.155 around 0.103.
        self.declare_parameter("grasp_z_above_min_m", 0.083)
        self.declare_parameter("grasp_z_above_max_m", 0.155)
        self.declare_parameter("world_name", "pick_place_desk")
        # Where the gripper center sits in the hand frame: fingertips hang ~0.10 m below
        # the panda_hand origin along its local +z (toward the object in a top-down grasp).
        self.declare_parameter("gripper_center_in_hand_z_m", 0.10)

        self._hand_frame = self.get_parameter("hand_frame").value
        self._planning_frame = self.get_parameter("planning_frame").value
        self._controller = self.get_parameter("gripper_controller").value
        self._graspable = list(self.get_parameter("graspable_models").value)
        self._grasp_xy_tol = float(self.get_parameter("grasp_xy_tol_m").value)
        self._grasp_z_min = float(self.get_parameter("grasp_z_above_min_m").value)
        self._grasp_z_max = float(self.get_parameter("grasp_z_above_max_m").value)

        self._cb_group = ReentrantCallbackGroup()
        self._width_lock = threading.Lock()
        self._current_width = 0.08
        self._attached_model: str | None = None
        self._grasp_target: str | None = None
        self._scene: SceneState | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._world = self.get_parameter("world_name").value
        self._gripper_center_z = float(self.get_parameter("gripper_center_in_hand_z_m").value)

        traj_action = f"/{self._controller}/follow_joint_trajectory"
        self._traj_client = rclpy.action.ActionClient(
            self, FollowJointTrajectory, traj_action, callback_group=self._cb_group,
        )

        # Snap the block to the gripper center just before attach so it ends up visually
        # gripped between the fingers, not pinned wherever it happened to be.
        self._set_pose_client = self.create_client(
            SetEntityPose, f"/world/{self._world}/set_pose", callback_group=self._cb_group,
        )

        # One attach/detach publisher pair per graspable model (bridged to gz topics).
        self._attach_pubs = {
            m: self.create_publisher(Empty, f"/grasp/{m}/attach", 10)
            for m in self._graspable
        }
        self._detach_pubs = {
            m: self.create_publisher(Empty, f"/grasp/{m}/detach", 10)
            for m in self._graspable
        }

        self.create_subscription(
            JointState, "/joint_states_hw", self._on_joint_state, 10,
        )
        self.create_subscription(
            String, "/gazebo_gripper/grasp_target", self._on_grasp_target, 10,
        )
        self.create_subscription(SceneState, "/scene_state", self._on_scene, 10)

        self._attached_pub = self.create_publisher(String, "/gazebo_gripper/attached_object", 10)

        self._move_server = ActionServer(
            self, Move, "/franka_gripper/move",
            execute_callback=self._execute_move,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self._grasp_server = ActionServer(
            self, Grasp, "/franka_gripper/grasp",
            execute_callback=self._execute_grasp,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )

        # Clear any joints left attached from a previous run once the bridge has connected
        # (a fresh node starts with _attached_model=None but gz may still hold a joint).
        self._startup_detach_timer = self.create_timer(2.0, self._startup_detach_once)

        self.get_logger().info(
            f"franka_gripper shim ready (controller={self._controller}, "
            f"physical DetachableJoint attach for {self._graspable})",
        )

    def _startup_detach_once(self) -> None:
        self._startup_detach_timer.cancel()
        for pub in self._detach_pubs.values():
            pub.publish(Empty())
        self.get_logger().info("startup: cleared any stale grasp joints")

    def _on_joint_state(self, msg: JointState) -> None:
        if FINGER_JOINT not in msg.name:
            return
        idx = msg.name.index(FINGER_JOINT)
        if idx >= len(msg.position):
            return
        with self._width_lock:
            self._current_width = joint_to_width(msg.position[idx])

    def _on_grasp_target(self, msg: String) -> None:
        self._grasp_target = msg.data.strip() or None

    def _on_scene(self, msg: SceneState) -> None:
        self._scene = msg

    def _scene_position(self, model_name: str) -> tuple[float, float, float] | None:
        """Find a graspable's pose in /scene_state by LABEL, not by id.

        `model_name` is the gz model name (e.g. red_block_01) used for attach/detach topics.
        /scene_state ids can differ from / drift away from the gz model name, so match on the
        label prefix instead: red_block_01 -> label "red_block". The scene has one object per
        label, so this is unambiguous and immune to id drift (which used to make grasp-select
        report "no scene pose" and abort the grasp)."""
        if self._scene is None:
            return None
        label = self._model_label(model_name)
        for obj in self._scene.objects:
            if obj.label != label:
                continue
            p = obj.pose.pose.position
            return (p.x, p.y, p.z)
        return None

    @staticmethod
    def _model_label(model_name: str) -> str:
        # Strip a trailing _NN id suffix: "red_block_01" -> "red_block".
        parts = model_name.rsplit("_", 1)
        if len(parts) == 2 and parts[1].isdigit():
            return parts[0]
        return model_name

    def _current_width_m(self) -> float:
        with self._width_lock:
            return self._current_width

    def _hand_position(self) -> tuple[float, float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._planning_frame,
                self._hand_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _select_grasp_model(self) -> str | None:
        """Pick the object the hand is over: horizontal distance + vertical band gate.

        The executor publishes the intended object on /gazebo_gripper/grasp_target; trust
        it if it still passes the alignment gate, otherwise fall back to nearest-by-xy.
        """
        hand = self._hand_position()
        if hand is None:
            return None

        candidates = self._graspable
        if self._grasp_target and self._grasp_target in self._graspable:
            candidates = [self._grasp_target]

        best_name: str | None = None
        best_xy = self._grasp_xy_tol
        for model_name in candidates:
            pos = self._scene_position(model_name)
            if pos is None:
                self.get_logger().warning(f"grasp-select {model_name}: no scene pose")
                continue
            z_above = hand[2] - pos[2]
            xy = math.hypot(hand[0] - pos[0], hand[1] - pos[1])
            self.get_logger().info(
                f"grasp-select {model_name}: hand={hand[0]:.3f},{hand[1]:.3f},{hand[2]:.3f} "
                f"obj={pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f} xy={xy:.3f}(tol {self._grasp_xy_tol}) "
                f"z_above={z_above:.3f}(band {self._grasp_z_min}..{self._grasp_z_max})",
            )
            if not (self._grasp_z_min <= z_above <= self._grasp_z_max):
                continue
            if xy < best_xy:
                best_xy = xy
                best_name = model_name
        if best_name is None:
            self.get_logger().warning("grasp-select: no object passed the alignment gate")
        return best_name

    def _spin_wait(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def _send_finger_width(self, target_width: float, speed: float, timeout_sec: float) -> bool:
        if not self._traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"/{self._controller}/follow_joint_trajectory 未就绪")
            return False

        target_joint = width_to_joint(target_width)
        current_joint = width_to_joint(self._current_width_m())
        delta = abs(target_joint - current_joint)
        duration_sec = max(0.25, delta / max(speed / 2.0, 0.01))

        traj = JointTrajectory()
        traj.joint_names = [FINGER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [target_joint]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9),
        )
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = self._traj_client.send_goal_async(goal)
        if not self._spin_wait(send_future, 5.0):
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        if not self._spin_wait(result_future, timeout_sec):
            goal_handle.cancel_goal_async()
            return False

        wrapped = result_future.result()
        if wrapped is None or wrapped.result.error_code != 0:
            return False

        with self._width_lock:
            self._current_width = target_width
        return True

    def _gripper_center_world(self) -> tuple[float, float, float] | None:
        """World position of the gripper center (hand origin + offset down its local z)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._planning_frame, self._hand_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3),
            )
        except TransformException:
            return None
        # Rotate local +z offset by the hand orientation into the planning frame.
        q = tf.transform.rotation
        # z-axis of the hand frame expressed in planning frame (3rd column of R(q)).
        zx = 2.0 * (q.x * q.z + q.w * q.y)
        zy = 2.0 * (q.y * q.z - q.w * q.x)
        zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        t = tf.transform.translation
        d = self._gripper_center_z
        return (t.x + zx * d, t.y + zy * d, t.z + zz * d)

    def _snap_to_gripper(self, model_name: str) -> None:
        center = self._gripper_center_world()
        if center is None or not self._set_pose_client.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = model_name
        req.entity.type = Entity.MODEL
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        req.pose = pose
        future = self._set_pose_client.call_async(req)
        self._spin_wait(future, 0.5)

    def _attach(self, model_name: str) -> None:
        """Snap the object to the gripper center, then create the physical fixed joint so
        it ends up gripped between the fingers rather than pinned at an offset."""
        self._snap_to_gripper(model_name)
        time.sleep(0.05)  # let the set_pose settle before pinning the joint
        self._attach_pubs[model_name].publish(Empty())
        self._attached_model = model_name
        self._attached_pub.publish(String(data=model_name))
        self.get_logger().info(f"attach joint -> {model_name} (snapped to gripper center)")

    def _detach(self) -> None:
        if self._attached_model and self._attached_model in self._detach_pubs:
            self._detach_pubs[self._attached_model].publish(Empty())
            self.get_logger().info(f"detach joint -> {self._attached_model}")
        self._attached_model = None
        self._grasp_target = None
        self._attached_pub.publish(String(data=""))

    def _detach_all(self) -> None:
        """Detach every graspable model unconditionally. gz DetachableJoint state can
        survive this node's restart (this node's _attached_model resets to None but the
        gz joint persists), leaving an object dangling from the arm. Opening the gripper
        must clear ALL joints, not just the one this instance remembers."""
        for model_name, pub in self._detach_pubs.items():
            pub.publish(Empty())
        if self._attached_model:
            self.get_logger().info(f"detach-all (was holding {self._attached_model})")
        self._attached_model = None
        self._grasp_target = None
        self._attached_pub.publish(String(data=""))

    def _execute_move(self, goal_handle):
        req = goal_handle.request
        target_width = float(req.width)
        speed = max(float(req.speed), 0.01)

        # Opening past the current width = releasing whatever is held. Detach ALL graspable
        # models so a stale joint (e.g. from before a node restart) can't leave an object
        # dangling from the arm.
        if target_width > self._current_width_m() + 0.005:
            self._detach_all()

        ok = self._send_finger_width(target_width, speed, timeout_sec=10.0)
        result = Move.Result()
        if ok:
            result.success = True
            goal_handle.succeed()
        else:
            result.success = False
            result.error = "gripper trajectory failed"
            goal_handle.abort()
        return result

    def _execute_grasp(self, goal_handle):
        req = goal_handle.request
        target_width = float(req.width)
        speed = max(float(req.speed), 0.01)

        ok = self._send_finger_width(target_width, speed, timeout_sec=10.0)
        result = Grasp.Result()
        if not ok:
            result.success = False
            result.error = "gripper trajectory failed"
            goal_handle.abort()
            return result

        model_name = self._select_grasp_model()
        if model_name is None:
            result.success = False
            result.error = "no graspable object aligned under hand"
            goal_handle.abort()
            return result

        self._attach(model_name)
        result.success = True
        goal_handle.succeed()
        return result


def main() -> None:
    rclpy.init()
    node = GazeboGripperSim()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
