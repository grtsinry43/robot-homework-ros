"""Structured MCP / inner-loop responses (DESIGN.md §4.3)."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from enum import Enum
from typing import Any


class ErrorCode(str, Enum):
    OBJECT_NOT_VISIBLE = "OBJECT_NOT_VISIBLE"
    UNKNOWN_OBJECT_ID = "UNKNOWN_OBJECT_ID"
    OUT_OF_REACH = "OUT_OF_REACH"
    MOTION_PLANNING_FAILED = "MOTION_PLANNING_FAILED"
    MOTION_COLLISION = "MOTION_COLLISION"
    GRASP_PLANNING_FAILED = "GRASP_PLANNING_FAILED"
    GRIPPER_SLIPPED = "GRIPPER_SLIPPED"
    SERVO_TIMEOUT = "SERVO_TIMEOUT"
    SERVO_ABORTED = "SERVO_ABORTED"
    INTERNAL_ERROR = "INTERNAL_ERROR"


DEFAULT_SUGGESTIONS: dict[ErrorCode, str] = {
    ErrorCode.OBJECT_NOT_VISIBLE: "调用 scan_scene 重新定位，或要求用户把物体移回工作区",
    ErrorCode.UNKNOWN_OBJECT_ID: "调用 scan_scene 或检查 id 拼写",
    ErrorCode.OUT_OF_REACH: "让用户移动物体到工作区中央",
    ErrorCode.MOTION_PLANNING_FAILED: "重试，或先 scan_scene 检查障碍",
    ErrorCode.MOTION_COLLISION: "先 abort，再 scan_scene",
    ErrorCode.GRASP_PLANNING_FAILED: "换一个物体或换一个角度",
    ErrorCode.GRIPPER_SLIPPED: "重新 pick_object",
    ErrorCode.SERVO_TIMEOUT: "重新 pick_object",
    ErrorCode.SERVO_ABORTED: "由 LLM 决定后续",
    ErrorCode.INTERNAL_ERROR: "终止任务，报告用户",
}


@dataclass
class ToolResponse:
    status: str
    payload: dict[str, Any]

    def to_json(self) -> str:
        return json.dumps(asdict(self) if False else {**{"status": self.status}, **self.payload}, ensure_ascii=False)


def ok(**payload: Any) -> str:
    return json.dumps({"status": "ok", **payload}, ensure_ascii=False)


def aborted(what: str) -> str:
    return json.dumps({"status": "aborted", "what": what}, ensure_ascii=False)


def failed(code: ErrorCode | str, reason: str, suggestion: str | None = None) -> str:
    if isinstance(code, ErrorCode):
        suggestion = suggestion or DEFAULT_SUGGESTIONS[code]
        code_str = code.value
    else:
        code_str = code
        suggestion = suggestion or DEFAULT_SUGGESTIONS[ErrorCode.INTERNAL_ERROR]
    return json.dumps(
        {
            "status": "failed",
            "code": code_str,
            "reason": reason,
            "suggestion": suggestion,
        },
        ensure_ascii=False,
    )


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
