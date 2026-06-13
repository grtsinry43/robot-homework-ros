"""MoveIt planning scene sync: world collision objects + attach/detach (DESIGN §5.4)."""

from __future__ import annotations

import time
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scene_state_msgs.msg import SceneState
from shape_msgs.msg import SolidPrimitive


# Approximate collision primitives for desk props (meters).
_LABEL_PRIMITIVES: dict[str, tuple[int, list[float]]] = {
    "red_block": (SolidPrimitive.BOX, [0.04, 0.04, 0.04]),
    "green_block": (SolidPrimitive.BOX, [0.04, 0.04, 0.04]),
    "blue_plate": (SolidPrimitive.BOX, [0.16, 0.16, 0.012]),
    "yellow_wall": (SolidPrimitive.BOX, [0.02, 0.12, 0.08]),
}

_DEFAULT_TOUCH_LINKS = (
    "panda_hand",
    "panda_leftfinger",
    "panda_rightfinger",
    "panda_link8",
)


class PlanningSceneHelper:
    """Apply diff updates to /apply_planning_scene."""

    def __init__(
        self,
        node: Node,
        *,
        planning_frame: str = "panda_link0",
        attach_link: str = "panda_hand",
        touch_links: Iterable[str] = _DEFAULT_TOUCH_LINKS,
    ) -> None:
        self._node = node
        self._planning_frame = planning_frame
        self._attach_link = attach_link
        self._touch_links = list(touch_links)
        self._cb_group = ReentrantCallbackGroup()
        self._client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=self._cb_group,
        )
        self._tracked_world_ids: set[str] = set()
        self._attached_id: str | None = None

    def ready(self) -> bool:
        return self._client.service_is_ready()

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def _apply(self, scene: PlanningScene, timeout_sec: float = 5.0) -> bool:
        if not self._client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warning("apply_planning_scene 未就绪")
            return False
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._client.call_async(req)
        if not self._wait_future(future, timeout_sec):
            self._node.get_logger().warning("apply_planning_scene call timed out")
            return False
        resp = future.result()
        if resp is None:
            self._node.get_logger().warning("apply_planning_scene returned no result")
            return False
        if not resp.success:
            self._node.get_logger().warning("apply_planning_scene resp.success=False")
        return resp.success

    @staticmethod
    def _primitive_for_label(label: str) -> tuple[int, list[float]]:
        return _LABEL_PRIMITIVES.get(label, (SolidPrimitive.BOX, [0.04, 0.04, 0.04]))

    def _make_world_object(
        self,
        object_id: str,
        label: str,
        pose: Pose,
        *,
        operation: int,
    ) -> CollisionObject:
        obj = CollisionObject()
        obj.id = object_id
        obj.operation = operation
        if operation == CollisionObject.REMOVE:
            return obj
        obj.header.frame_id = self._planning_frame
        obj.header.stamp = self._node.get_clock().now().to_msg()
        prim_type, dims = self._primitive_for_label(label)
        prim = SolidPrimitive()
        prim.type = prim_type
        prim.dimensions = list(dims)
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        return obj

    def sync_world_from_scene(
        self,
        scene: SceneState | None,
        *,
        exclude_ids: Iterable[str] = (),
    ) -> bool:
        """Add/update table collision objects from /scene_state (diff)."""
        if scene is None:
            return True
        exclude = set(exclude_ids)
        desired: dict[str, tuple[str, Pose]] = {}
        for obj in scene.objects:
            if obj.id in exclude or obj.id == self._attached_id:
                continue
            if "wall" in obj.label:
                continue
            desired[obj.id] = (obj.label, obj.pose.pose)

        scene_diff = PlanningScene()
        scene_diff.is_diff = True

        for object_id in self._tracked_world_ids - desired.keys():
            scene_diff.world.collision_objects.append(
                self._make_world_object(object_id, "", Pose(), operation=CollisionObject.REMOVE),
            )

        for object_id, (label, pose) in desired.items():
            scene_diff.world.collision_objects.append(
                self._make_world_object(object_id, label, pose, operation=CollisionObject.ADD),
            )

        if not scene_diff.world.collision_objects:
            self._tracked_world_ids = set(desired.keys())
            return True

        ok = self._apply(scene_diff)
        if ok:
            self._tracked_world_ids = set(desired.keys())
        return ok

    def remove_world_object(self, object_id: str) -> bool:
        """Drop a single object from the collision world (e.g. the grasp target right
        before the final vertical descent, so the hand can reach it)."""
        scene_diff = PlanningScene()
        scene_diff.is_diff = True
        scene_diff.world.collision_objects = [
            self._make_world_object(object_id, "", Pose(), operation=CollisionObject.REMOVE),
        ]
        ok = self._apply(scene_diff)
        if ok:
            self._tracked_world_ids.discard(object_id)
        return ok

    def attach_to_hand(self, object_id: str, label: str, pose_in_hand: Pose) -> bool:
        """Attach grasped object to hand; remove it from world collision."""
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = self._attach_link
        co.header.stamp = self._node.get_clock().now().to_msg()
        co.operation = CollisionObject.ADD
        prim_type, dims = self._primitive_for_label(label)
        prim = SolidPrimitive()
        prim.type = prim_type
        prim.dimensions = list(dims)
        co.primitives.append(prim)
        co.primitive_poses.append(pose_in_hand)

        aco = AttachedCollisionObject()
        aco.link_name = self._attach_link
        aco.object = co
        aco.touch_links = list(self._touch_links)

        # NOTE: do NOT also push a world REMOVE for object_id here. By the time we attach, the
        # grasp target was already removed from the world (executor removes it before the
        # final descent), so a second REMOVE of a non-existent id made MoveIt reject the whole
        # diff (apply_planning_scene resp.success=False) — which left attach failing every
        # time and poisoned multi-step plans. The AttachedCollisionObject ADD carries its own
        # geometry, so it doesn't need the object to pre-exist in the world.
        scene_diff = PlanningScene()
        scene_diff.is_diff = True
        scene_diff.robot_state.is_diff = True
        scene_diff.robot_state.attached_collision_objects = [aco]

        ok = self._apply(scene_diff)
        if ok:
            self._attached_id = object_id
            self._tracked_world_ids.discard(object_id)
        return ok

    def detach_to_world(self, object_id: str, label: str, world_pose: Pose) -> bool:
        """Detach from hand and re-add object at world pose."""
        remove_attached = AttachedCollisionObject()
        remove_attached.link_name = self._attach_link
        remove_attached.object.id = object_id
        remove_attached.object.operation = CollisionObject.REMOVE

        add_world = self._make_world_object(
            object_id, label, world_pose, operation=CollisionObject.ADD,
        )

        scene_diff = PlanningScene()
        scene_diff.is_diff = True
        scene_diff.robot_state.is_diff = True
        scene_diff.robot_state.attached_collision_objects = [remove_attached]
        scene_diff.world.collision_objects = [add_world]

        ok = self._apply(scene_diff)
        if ok:
            if self._attached_id == object_id:
                self._attached_id = None
            self._tracked_world_ids.add(object_id)
        return ok

    def attached_object_id(self) -> str | None:
        return self._attached_id
