"""Shared MCP-layer checks before dispatching to the inner loop."""

from __future__ import annotations

from scene_state_msgs.msg import ObjectPose, SceneState

from .errors import ErrorCode, failed
from .offset_resolver import VALID_OFFSETS, resolve_offset
from .workspace_envelope import DEFAULT_ENVELOPE


def find_object(scene: SceneState | None, object_id: str) -> ObjectPose | None:
    if scene is None:
        return None
    for obj in scene.objects:
        if obj.id == object_id:
            return obj
    return None


def validate_pick_target(scene: SceneState | None, object_id: str) -> str | None:
    if not object_id or not object_id.strip():
        return failed(ErrorCode.UNKNOWN_OBJECT_ID, "object id 不能为空")
    obj = find_object(scene, object_id.strip())
    if obj is None:
        return failed(
            ErrorCode.UNKNOWN_OBJECT_ID,
            f"blackboard 无 {object_id}",
        )
    x, y, z = obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z
    return DEFAULT_ENVELOPE.check_or_error(x, y, z)


def validate_place_target(scene: SceneState | None, target_id: str, offset: str) -> str | None:
    if offset not in VALID_OFFSETS:
        return failed(
            ErrorCode.INTERNAL_ERROR,
            f"offset 必须是 {sorted(VALID_OFFSETS)} 之一",
        )
    if not target_id or not target_id.strip():
        return failed(ErrorCode.UNKNOWN_OBJECT_ID, "target_id 不能为空")
    obj = find_object(scene, target_id.strip())
    if obj is None:
        return failed(
            ErrorCode.UNKNOWN_OBJECT_ID,
            f"blackboard 无 {target_id}",
        )
    place = resolve_offset(obj.pose, offset)
    px, py, pz = place.pose.position.x, place.pose.position.y, place.pose.position.z
    return DEFAULT_ENVELOPE.check_or_error(px, py, pz)
