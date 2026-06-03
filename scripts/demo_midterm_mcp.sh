#!/usr/bin/env bash
# Mid-term MCP acceptance demo (DESIGN.md §4). Requires Phase 2 unified stack.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"

EXECUTE="${1:-}"
SCENARIO="${2:-all}"

source_ros

_run() {
  local extra=()
  if [[ "$EXECUTE" == "--execute" ]]; then
    extra=(--execute)
  fi
  if [[ "$SCENARIO" == "all" ]]; then
    for n in 1 2 3 4 5; do
      echo ""
      python3 "$ROOT/scripts/mcp_demo_cli.py" scenario "$n" "${extra[@]}" || true
      if [[ -t 0 ]]; then
        read -r -p "按 Enter 继续下一场景…" _ || true
      else
        sleep 2
      fi
    done
  else
    python3 "$ROOT/scripts/mcp_demo_cli.py" scenario "$SCENARIO" "${extra[@]}"
  fi
}

echo "==> MCP 中期演示"
echo "    前置: ./scripts/run_in_container.sh start-phase2-unified"
echo "    模式: EXECUTE=${EXECUTE:-dry-run}  SCENARIO=${SCENARIO}"
echo ""

if ! ros2 action list 2>/dev/null | grep -q /pick_place/pick_object; then
  echo "ERROR: pick_place 栈未就绪。请先 start-phase2-unified" >&2
  exit 1
fi

_run

echo ""
echo "==> 完成。Cursor MCP 联调: ./scripts/run_in_container.sh start-mcp"
