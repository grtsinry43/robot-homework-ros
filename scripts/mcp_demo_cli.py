#!/usr/bin/env python3
"""Mid-term MCP demo CLI — same tool contract as mcp_pick_place_brain.py (no MCP stdio)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_PKG = _REPO / "ros2_ws" / "src" / "panda_pick_place"
_MCP = _REPO / "ros2_ws"
for p in (_PKG, _MCP):
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import rclpy  # noqa: E402
from mcp_pick_place_brain import PickPlaceMcpBridge  # noqa: E402
from panda_pick_place.mcp_intent import ToolCall, resolve_pick_place_intent  # noqa: E402


def _print_json(raw: str) -> None:
    try:
        print(json.dumps(json.loads(raw), ensure_ascii=False, indent=2))
    except json.JSONDecodeError:
        print(raw)


def _scene_ids(node: PickPlaceMcpBridge) -> list[str]:
    if node._latest_scene is None:
        return []
    return [o.id for o in node._latest_scene.objects]


def _run_tool(node: PickPlaceMcpBridge, call: ToolCall) -> str:
    print(f"\n>>> {call.tool}({json.dumps(call.args, ensure_ascii=False)})")
    if call.tool == "scan_scene":
        out = node.scan_scene()
    elif call.tool == "pick_object":
        out = node.pick_object(call.args["id"])
    elif call.tool == "place_at":
        out = node.place_at(call.args["target_id"], call.args["offset"])
    elif call.tool == "abort_current_task":
        out = node.abort_current_task()
    elif call.tool == "execute_arm_move":
        out = node.execute_arm_move(
            float(call.args["x"]), float(call.args["y"]), float(call.args["z"]),
        )
    else:
        out = json.dumps({"status": "failed", "reason": f"unknown tool {call.tool}"})
    _print_json(out)
    return out


def _spin_once(node: PickPlaceMcpBridge, sec: float = 0.3) -> None:
    import time
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser(description="MCP mid-term demo CLI")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", help="scan_scene()")
    p_pick = sub.add_parser("pick", help="pick_object(id)")
    p_pick.add_argument("id")
    p_place = sub.add_parser("place", help="place_at(target_id, offset)")
    p_place.add_argument("target_id")
    p_place.add_argument("offset", default="above", nargs="?")
    sub.add_parser("abort", help="abort_current_task()")

    p_nl = sub.add_parser("nl", help="NL intent → tool plan (LLM stand-in)")
    p_nl.add_argument("text")
    p_nl.add_argument("--execute", action="store_true", help="Run resolved plan")

    p_sc = sub.add_parser("scenario", help="Run scripted mid-term scenario 1-5")
    p_sc.add_argument("number", type=int, choices=[1, 2, 3, 4, 5])
    p_sc.add_argument("--execute", action="store_true", help="Execute pick/place (scenario 1/4)")

    args = parser.parse_args()

    rclpy.init()
    node = PickPlaceMcpBridge()
    _spin_once(node, 1.0)

    try:
        if args.cmd == "scan":
            _run_tool(node, ToolCall("scan_scene", {}))
        elif args.cmd == "pick":
            _run_tool(node, ToolCall("pick_object", {"id": args.id}))
        elif args.cmd == "place":
            _run_tool(node, ToolCall("place_at", {"target_id": args.target_id, "offset": args.offset}))
        elif args.cmd == "abort":
            _run_tool(node, ToolCall("abort_current_task", {}))
        elif args.cmd == "nl":
            node.scan_scene()
            _spin_once(node, 0.5)
            plan, note = resolve_pick_place_intent(args.text, _scene_ids(node))
            if note:
                print(f"\n[clarify] {note}")
            print("\n[plan]")
            for step in plan:
                print(f"  - {step.tool}({step.args})")
            if args.execute:
                for step in plan:
                    out = _run_tool(node, step)
                    if '"status": "failed"' in out or '"status": "aborted"' in out:
                        break
                    _spin_once(node, 0.3)
        elif args.cmd == "scenario":
            _run_scenario(node, args.number, execute=args.execute)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _run_scenario(node: PickPlaceMcpBridge, n: int, *, execute: bool) -> None:
    if n == 1:
        print("=== 场景1：基础语义抓放 ===")
        steps = [
            ToolCall("scan_scene", {}),
            ToolCall("pick_object", {"id": "red_block_01"}),
            ToolCall("place_at", {"target_id": "blue_plate_01", "offset": "above"}),
        ]
    elif n == 2:
        print("=== 场景2：同义表达鲁棒性（LLM 归一化示意）===")
        phrases = [
            "把红色方块放到蓝色盘子里",
            "把红色的那个放进盘子",
            "把红块放到蓝盘上",
        ]
        for phrase in phrases:
            print(f"\n--- 用户: {phrase}")
            node.scan_scene()
            _spin_once(node, 0.3)
            plan, note = resolve_pick_place_intent(phrase, _scene_ids(node))
            if note:
                print(f"[clarify] {note}")
            for step in plan:
                print(f"  → {step.tool}({step.args})")
        return
    elif n == 3:
        print("=== 场景3：参数与安全检查 ===")
        checks = [
            ("UNKNOWN_OBJECT_ID", ToolCall("pick_object", {"id": "purple_ball_01"})),
            ("bad offset", ToolCall("place_at", {"target_id": "blue_plate_01", "offset": "diagonal"})),
            ("OUT_OF_REACH", ToolCall("place_at", {"target_id": "outside_table", "offset": "above"})),
            ("OUT_OF_REACH (debug move)", ToolCall("execute_arm_move", {"x": 0.9, "y": 0.0, "z": 0.3})),
        ]
        node.scan_scene()
        _spin_once(node, 0.3)
        for title, step in checks:
            print(f"\n--- 期望: {title}")
            _run_tool(node, step)
        print("\n[说明] 「把它放过去」类缺参指令由 LLM 层澄清，不应直接 place_at。")
        return
    elif n == 4:
        print("=== 场景4：黄色障碍物（设计 + 可见墙）===")
        node.scan_scene()
        _spin_once(node, 0.3)
        _print_json(node.scan_scene())
        print(
            "\n[说明] 黄墙在 Gazebo 可见；完整避障需将障碍物写入 MoveIt planning scene。\n"
            "       LLM 只下达 pick/place 语义；几何避障由 MoveIt2 负责（中期为设计方案）。"
        )
        if execute:
            steps = [
                ToolCall("pick_object", {"id": "red_block_01"}),
                ToolCall("place_at", {"target_id": "blue_plate_01", "offset": "right_of"}),
            ]
            for step in steps:
                _run_tool(node, step)
                _spin_once(node, 0.3)
        return
    else:
        print("=== 场景5：语音输入（文字链路演示）===")
        text = "把红色方块放到蓝色盘子里"
        print(f"[Whisper 假设输出] {text}")
        plan, note = resolve_pick_place_intent(text, _scene_ids(node))
        if note:
            print(f"[clarify] {note}")
        for step in plan:
            print(f"  → {step.tool}({step.args})")
        if execute:
            for step in plan:
                _run_tool(node, step)
        return

    for step in steps:
        if not execute and step.tool != "scan_scene":
            print(f"\n>>> {step.tool}({json.dumps(step.args, ensure_ascii=False)})  [dry-run]")
            continue
        out = _run_tool(node, step)
        if execute and '"status": "failed"' in out:
            break
        _spin_once(node, 0.5)


if __name__ == "__main__":
    raise SystemExit(main())
