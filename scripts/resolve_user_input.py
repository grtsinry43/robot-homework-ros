#!/usr/bin/env python3
"""CLI helper: text or Whisper speech → stdout (for LLM client integration)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
_PKG = _REPO_ROOT / "ros2_ws" / "src" / "panda_pick_place"
if _PKG.is_dir() and str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from panda_pick_place.speech_input import resolve_user_input  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Resolve user input (text or speech)")
    parser.add_argument("--mode", choices=["text", "speech"], default="text")
    parser.add_argument("--text", help="Direct text input")
    parser.add_argument("--audio", help="Audio file path for speech mode")
    parser.add_argument("--whisper-model", default="small", help="tiny|base|small|medium")
    parser.add_argument("--json", action="store_true", help="Output JSON")
    args = parser.parse_args()

    try:
        result = resolve_user_input(
            text=args.text,
            audio_path=args.audio,
            mode=args.mode,
            whisper_model=args.whisper_model,
        )
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(str(exc), file=sys.stderr)
        return 1

    if args.json:
        payload = {"text": result.text, "mode": result.mode, "backend": result.backend}
        print(json.dumps(payload, ensure_ascii=False))
    else:
        print(result.text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
