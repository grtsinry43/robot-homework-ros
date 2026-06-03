"""Lightweight NL → tool plan for mid-term demo (LLM stand-in)."""

from __future__ import annotations

import re
from dataclasses import dataclass


@dataclass
class ToolCall:
    tool: str
    args: dict[str, str]


def _has_red(text: str) -> bool:
    return bool(re.search(r"红|red", text, re.I))


def _has_blue(text: str) -> bool:
    return bool(re.search(r"蓝|blue|盘|托盘", text, re.I))


def _has_green(text: str) -> bool:
    return bool(re.search(r"绿|green", text, re.I))


def resolve_pick_place_intent(user_text: str, scene_ids: list[str]) -> tuple[list[ToolCall], str | None]:
    """Map Chinese NL to scan → pick → place. Returns (plan, clarification)."""
    text = user_text.strip()
    if not text:
        return [], "请说明要抓什么、放到哪里"

    vague = bool(re.search(r"把它|放过去|那个东西|那边", text))
    if vague and not (_has_red(text) or _has_green(text) or _has_blue(text)):
        return [], "指令缺少明确物体，请先 scan_scene 或说明颜色/目标"

    if re.search(r"桌子外|外面|桌外|off.?table", text, re.I):
        return [
            ToolCall("scan_scene", {}),
            ToolCall("pick_object", {"id": _pick_id(scene_ids, prefer="red")}),
            ToolCall("place_at", {"target_id": "outside_table", "offset": "above"}),
        ], None

    if re.search(r"紫|purple", text, re.I):
        return [
            ToolCall("scan_scene", {}),
            ToolCall("pick_object", {"id": "purple_ball_01"}),
        ], None

    pick_id = _pick_id(scene_ids, prefer="red" if _has_red(text) else "green")
    place_id = _place_id(scene_ids, prefer="blue")
    offset = "right_of" if re.search(r"旁边|右侧|右边", text) else "above"

    if not pick_id:
        return [ToolCall("scan_scene", {})], "需要先 scan_scene 确认抓取物 id"
    if not place_id:
        return [ToolCall("scan_scene", {}), ToolCall("pick_object", {"id": pick_id})], "需要先 scan_scene 确认放置目标 id"

    return [
        ToolCall("scan_scene", {}),
        ToolCall("pick_object", {"id": pick_id}),
        ToolCall("place_at", {"target_id": place_id, "offset": offset}),
    ], None


def _pick_id(scene_ids: list[str], prefer: str) -> str | None:
    label = f"{prefer}_block"
    for oid in scene_ids:
        if label in oid:
            return oid
    for oid in scene_ids:
        if "block" in oid or "ball" in oid:
            return oid
    return scene_ids[0] if scene_ids else None


def _place_id(scene_ids: list[str], prefer: str) -> str | None:
    for oid in scene_ids:
        if prefer in oid and ("plate" in oid or "ball" in oid):
            return oid
    for oid in scene_ids:
        if "plate" in oid or ("ball" in oid and "blue" in oid):
            return oid
    return None
