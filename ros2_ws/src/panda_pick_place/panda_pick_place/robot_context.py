"""Runtime observation summary for the MCP/LLM outer loop."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node
from scene_state_msgs.msg import SceneState

from .errors import utc_now_iso
from .workspace_envelope import DEFAULT_ENVELOPE


class RobotContextBuilder:
    """Build a read-only snapshot of robot-facing runtime state."""

    def __init__(self, node: Node, *, stale_after_sec: float = 10.0) -> None:
        self._node = node
        self._stale_after_sec = float(stale_after_sec)

    def build(
        self,
        *,
        scene: SceneState | None,
        readiness: dict[str, bool],
        active_task: str | None = None,
        last_error: dict[str, Any] | None = None,
        recent_events: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        scene_summary = self._summarize_scene(scene)
        executor_ready = bool(
            readiness.get("pick_action")
            and readiness.get("place_action")
            and readiness.get("abort_service")
        )
        ready_for_pick_place = bool(
            scene_summary["fresh"]
            and scene_summary["object_count"] > 0
            and readiness.get("perception_service")
            and executor_ready
            and readiness.get("moveit_action")
        )

        return {
            "generated_at": utc_now_iso(),
            "scene": scene_summary,
            "readiness": {
                "perception": {
                    "trigger_scan_service": bool(readiness.get("perception_service")),
                },
                "executor": {
                    "pick_action": bool(readiness.get("pick_action")),
                    "place_action": bool(readiness.get("place_action")),
                    "abort_service": bool(readiness.get("abort_service")),
                    "ready": executor_ready,
                },
                "motion": {
                    "moveit_move_action": bool(readiness.get("moveit_action")),
                    "gripper_actions": bool(readiness.get("gripper_actions")),
                    "gripper_note": (
                        "Required for real grasping; demo configs may enable allow_gripper_skip."
                    ),
                },
                "ready_for_pick_place": ready_for_pick_place,
            },
            "executor": {
                "active_task_via_mcp": active_task,
                "busy": True if active_task else None,
                "busy_observation": (
                    "MCP bridge observes only tasks it dispatched; executor has no status topic yet."
                ),
            },
            "workspace_envelope": self._workspace_envelope(),
            "last_error": last_error,
            "recent_events": recent_events or [],
            "recommended_next_step": self._recommend_next_step(
                scene_summary,
                readiness,
                executor_ready=executor_ready,
                active_task=active_task,
            ),
        }

    def _summarize_scene(self, scene: SceneState | None) -> dict[str, Any]:
        if scene is None:
            return {
                "available": False,
                "fresh": False,
                "age_sec": None,
                "stale_after_sec": self._stale_after_sec,
                "object_count": 0,
                "objects": [],
            }

        age_sec = self._age_sec(scene.stamp)
        fresh = age_sec is not None and age_sec <= self._stale_after_sec
        objects = []
        for obj in scene.objects:
            pos = obj.pose.pose.position
            objects.append({
                "id": obj.id,
                "label": obj.label,
                "confidence": round(float(obj.confidence), 3),
                "frame_id": obj.pose.header.frame_id,
                "last_seen_age_sec": self._age_sec(obj.last_seen_at),
                "inside_workspace": DEFAULT_ENVELOPE.contains(pos.x, pos.y, pos.z),
            })

        return {
            "available": True,
            "fresh": fresh,
            "age_sec": age_sec,
            "stale_after_sec": self._stale_after_sec,
            "object_count": len(objects),
            "objects": objects,
        }

    def _age_sec(self, stamp) -> float | None:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        now = self._node.get_clock().now()
        age = (now - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9
        return round(max(0.0, age), 3)

    @staticmethod
    def _workspace_envelope() -> dict[str, float | str]:
        return {
            "frame_id": "panda_link0",
            "x_min": DEFAULT_ENVELOPE.x_min,
            "x_max": DEFAULT_ENVELOPE.x_max,
            "y_min": DEFAULT_ENVELOPE.y_min,
            "y_max": DEFAULT_ENVELOPE.y_max,
            "z_min": DEFAULT_ENVELOPE.z_min,
            "z_max": DEFAULT_ENVELOPE.z_max,
        }

    @staticmethod
    def _recommend_next_step(
        scene_summary: dict[str, Any],
        readiness: dict[str, bool],
        *,
        executor_ready: bool,
        active_task: str | None,
    ) -> str:
        if active_task:
            return "wait_for_current_task_or_call_abort_current_task"
        if not readiness.get("perception_service"):
            return "start_pick_place_launch_or_restart_perception"
        if not scene_summary["available"]:
            return "call_scan_scene"
        if not scene_summary["fresh"]:
            return "call_scan_scene"
        if scene_summary["object_count"] == 0:
            return "call_scan_scene_or_move_objects_into_view"
        if not executor_ready:
            return "start_pick_place_executor"
        if not readiness.get("moveit_action"):
            return "start_moveit_move_group_before_pick_place"
        if not readiness.get("gripper_actions"):
            return "real_grasping_requires_franka_gripper_or_demo_skip"
        return "ready_for_pick_place"
