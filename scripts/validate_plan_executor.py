#!/usr/bin/env python3
"""Validate the short linear plan executor contract without ROS or Gazebo."""

from __future__ import annotations

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "ros2_ws" / "src" / "panda_pick_place"

if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from panda_pick_place.action_library import load_action_library  # noqa: E402
from panda_pick_place.plan_executor import (  # noqa: E402
    MAX_PLAN_STEPS,
    PlanValidationError,
    parse_plan,
)


def _expect_valid(name: str, plan: list[dict], library: dict) -> None:
    steps = parse_plan(json.dumps(plan), library)
    if len(steps) != len(plan):
        raise AssertionError(f"{name}: expected {len(plan)} steps, got {len(steps)}")
    print(f"[PASS] valid: {name}")


def _expect_invalid(name: str, plan_json: str, library: dict, expected: str) -> None:
    try:
        parse_plan(plan_json, library)
    except PlanValidationError as exc:
        message = str(exc)
        if expected not in message:
            raise AssertionError(f"{name}: expected {expected!r} in {message!r}") from exc
        print(f"[PASS] invalid: {name}")
        return
    raise AssertionError(f"{name}: expected PlanValidationError")


def main() -> int:
    library = load_action_library()

    _expect_valid(
        "pick then place",
        [
            {"tool": "pick_object", "args": {"id": "red_block_01"}},
            {"tool": "place_at", "args": {"target_id": "blue_plate_01", "offset": "above"}},
        ],
        library,
    )
    _expect_valid(
        "context and scan",
        [
            {"tool": "get_robot_context", "args": {}},
            {"tool": "scan_scene", "args": {}},
        ],
        library,
    )

    _expect_invalid(
        "debug tool rejected",
        json.dumps([{"tool": "execute_arm_move", "args": {"x": 0.4, "y": 0.0, "z": 0.3}}]),
        library,
        "not allowed",
    )
    _expect_invalid(
        "recursive plan rejected",
        json.dumps([{"tool": "execute_plan", "args": {"plan_json": "[]"}}]),
        library,
        "not allowed",
    )
    _expect_invalid(
        "bad enum rejected",
        json.dumps([{"tool": "place_at", "args": {"target_id": "blue_plate_01", "offset": "diagonal"}}]),
        library,
        "must be one of",
    )
    _expect_invalid(
        "missing required argument rejected",
        json.dumps([{"tool": "pick_object", "args": {}}]),
        library,
        "missing required argument",
    )
    _expect_invalid(
        "unknown argument rejected",
        json.dumps([{"tool": "scan_scene", "args": {"force": True}}]),
        library,
        "unknown arguments",
    )
    _expect_invalid(
        "too many steps rejected",
        json.dumps([{"tool": "scan_scene", "args": {}} for _ in range(MAX_PLAN_STEPS + 1)]),
        library,
        "maximum",
    )
    _expect_invalid("non-json rejected", "not json", library, "valid JSON")

    print("[PASS] plan executor validation checks complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
