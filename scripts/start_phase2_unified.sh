#!/usr/bin/env bash
# Phase 2 推荐入口：Gazebo 单窗口（桌面 + 色块 + Franka）+ MoveIt 内环，默认不启 RViz。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT/scripts/start_phase2_rviz.sh" "$@"
