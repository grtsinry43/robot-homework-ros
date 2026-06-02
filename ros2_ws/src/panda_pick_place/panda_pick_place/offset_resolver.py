"""Semantic placement offsets (DESIGN.md §4.2, §8 D5)."""

from __future__ import annotations

from geometry_msgs.msg import Pose, PoseStamped

VALID_OFFSETS = frozenset({"above", "left_of", "right_of", "front_of", "behind"})

# Small displacements in panda_link0 (meters); tune during Gazebo demo.
_OFFSET_DELTA = {
    "above": (0.0, 0.0, 0.12),
    "left_of": (0.0, 0.08, 0.05),
    "right_of": (0.0, -0.08, 0.05),
    "front_of": (0.08, 0.0, 0.05),
    "behind": (-0.08, 0.0, 0.05),
}


def resolve_offset(base: PoseStamped, offset: str) -> PoseStamped:
    if offset not in VALID_OFFSETS:
        raise ValueError(f"unsupported offset: {offset}")

    dx, dy, dz = _OFFSET_DELTA[offset]
    out = PoseStamped()
    out.header = base.header
    out.pose = Pose()
    out.pose.position.x = base.pose.position.x + dx
    out.pose.position.y = base.pose.position.y + dy
    out.pose.position.z = base.pose.position.z + dz
    out.pose.orientation = base.pose.orientation
    return out
