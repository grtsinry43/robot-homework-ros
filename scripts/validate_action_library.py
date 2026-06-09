#!/usr/bin/env python3
"""Validate the Panda pick-place atomic action library contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "ros2_ws" / "src" / "panda_pick_place"
DEFAULT_LIBRARY = PKG_SRC / "config" / "atomic_actions.json"

if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from panda_pick_place.action_library import load_action_library, render_action_library  # noqa: E402


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_LIBRARY
    try:
        library = load_action_library(path)
    except Exception as exc:  # noqa: BLE001
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1

    print(f"[PASS] {path}")
    print(f"==> {len(library['actions'])} actions")
    for action in library["actions"]:
        print(f"  - {action['name']} ({action['status']})")
    print("")
    print(render_action_library(library))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
