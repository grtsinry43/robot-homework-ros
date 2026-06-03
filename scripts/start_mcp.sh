#!/usr/bin/env bash
# MCP server must use stdio — attach via Cursor (config/mcp_client.docker.json), not background.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/stack_common.sh
source "$ROOT/scripts/lib/stack_common.sh"

source_ros

if ! ros2 action list 2>/dev/null | grep -q /pick_place/pick_object; then
  echo "WARN: pick_place 栈未就绪，请先 start-phase2-unified" >&2
fi

echo "MCP 通过 stdio 运行，请用 Cursor 加载:"
echo "  $ROOT/config/mcp_client.docker.json"
echo ""
echo "容器内等价命令（CLI 演示，无需 MCP 客户端）:"
echo "  python3 /root/scripts/mcp_demo_cli.py scan"
echo "  python3 /root/scripts/mcp_demo_cli.py scenario 1 --execute"
echo "  bash /root/scripts/demo_midterm_mcp.sh --execute 1"
echo ""
echo "手动前台启动 MCP（调试）:"
echo "  python3 /root/ros2_ws/mcp_pick_place_brain.py"
