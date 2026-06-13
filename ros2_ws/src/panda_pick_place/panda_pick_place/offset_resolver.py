"""Semantic placement offsets (DESIGN.md §4.2, §8 D5)."""

from __future__ import annotations

from geometry_msgs.msg import Pose, PoseStamped

VALID_OFFSETS = frozenset({"above", "left_of", "right_of", "front_of", "behind"})

# Small displacements in panda_link0 (meters); tune during Gazebo demo.
# NOTE on "above": this dz is the RELEASE height of the held-object CENTER over the
# target's REPORTED center. It is NOT a clearance offset — the block is released here,
# so it must end up resting on the plate, not free-falling. The full release chain is:
#   block_center ≈ link8_z − gripper_center_offset(0.10);  link8_z = pz + ee_grasp_offset
# so block_center ≈ pz − 0.10 + 0.103 ≈ pz; block_bottom ≈ pz − 0.02.
# Perception now snaps object z to the known layout height (plate ~0.055), so the reported
# pz is honest. block_bottom ≈ pz − 0.02; the plate top is ~0.06, so dz ≈ +0.025 lands the
# block bottom right on the plate surface for a gentle settle (not a free-fall).
# dz is the gentle-settle release height of the block center over the reference's reported
# center. History of tuning the lateral (table) placements:
#   0.05 -> too high: block fell ~2.5 cm onto the bare flat table, bounced and rolled far.
#   0.01 -> too low: the hand descended so far it poked/pressed into the block, and ended up
#           in contact with it (panda_hand vs block collision, blocking the next motion).
# 0.025 is the same gentle value "above" uses for the plate (verified: no bounce, no poke).
# The reference block center is ~0.06 (table height), so +0.025 sets a soft drop that settles
# the placed block next to it without the hand clipping it.
_OFFSET_DELTA = {
    "above": (0.0, 0.0, 0.025),
    "left_of": (0.0, 0.08, 0.025),
    "right_of": (0.0, -0.08, 0.025),
    "front_of": (0.08, 0.0, 0.025),
    "behind": (-0.08, 0.0, 0.025),
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
