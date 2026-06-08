#!/usr/bin/env bash
# Run stack scripts on the host via docker compose.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CMD="${1:-}"
shift || true

case "$CMD" in
  start-gazebo-desk)
    xhost +local:docker 2>/dev/null || true
    docker compose up -d
    docker compose exec -T ros2-gazebo bash /root/scripts/start_gazebo_desk_gui.sh "$@"
    ;;
  start-phase01)
    xhost +local:docker 2>/dev/null || true
    docker compose up -d
    docker compose exec -T -e PHASE01_USE_GUI="${PHASE01_USE_GUI:-}" ros2-gazebo bash /root/scripts/start_phase01.sh "$@"
    ;;
  start-phase2-rviz)
    xhost +local:docker 2>/dev/null || true
    docker compose up -d
    docker compose exec -T ros2-gazebo bash /root/scripts/start_phase2_rviz.sh "$@"
    ;;
  start-phase2-gazebo)
    xhost +local:docker 2>/dev/null || true
    docker compose up -d
    docker compose exec -T ros2-gazebo bash /root/scripts/start_phase2_gazebo.sh "$@"
    ;;
  start-phase2-unified)
    xhost +local:docker 2>/dev/null || true
    docker compose up -d
    docker compose exec -T ros2-gazebo bash /root/scripts/start_phase2_unified.sh "$@"
    ;;
  stop)
    docker compose exec -T ros2-gazebo bash /root/scripts/stop_stack.sh 2>/dev/null || true
    ;;
  verify-phase0)
    docker compose exec -T ros2-gazebo bash /root/scripts/verify_phase0.sh
    ;;
  verify-phase1)
    docker compose exec -T ros2-gazebo bash /root/scripts/verify_perception.sh
    ;;
  verify-phase2)
    docker compose exec -T ros2-gazebo bash /root/scripts/verify_phase2.sh
    ;;
  verify-gazebo-scene)
    docker compose exec -T ros2-gazebo bash /root/scripts/verify_gazebo_scene.sh
    ;;
  smoke)
    docker compose exec -T ros2-gazebo bash /root/scripts/smoke_pick_place.sh
    ;;
  demo-mcp)
    docker compose exec -T ros2-gazebo bash /root/scripts/demo_midterm_mcp.sh "$@"
    ;;
  start-mcp)
    docker compose exec -T ros2-gazebo bash /root/scripts/start_mcp.sh
    ;;
  shell)
    xhost +local:docker 2>/dev/null || true
    docker compose exec ros2-gazebo bash
    ;;
  *)
    cat <<EOF
Usage: $0 <command>

  start-gazebo-desk   Gazebo desk only (fix empty scene: run this first)
  start-phase01     Gazebo desk + perception + executor (Phase 0+1)
  start-phase2-rviz MoveIt RViz demo + servo + Phase 0+1 perception stack
  start-phase2-gazebo Franka Gazebo + camera + pick_place (experimental)
  start-phase2-unified Gazebo desk+Franka + MoveIt inner loop (one scene GUI)
  stop              Stop all background stack processes in container
  verify-phase0     Camera topics + TF checks
  verify-phase1     trigger_scan + /scene_state
  verify-phase2     move_action + servo + executor readiness
  smoke             ros2 action pick/place smoke (stack must be up)
  demo-mcp [opts]   Mid-term MCP scenarios (see prompts/midterm_demo.md)
  start-mcp         Hint: run MCP via config/mcp_client.docker.json in Cursor
  shell             Interactive bash in container

Examples:
  $0 start-phase01
  $0 start-phase2-rviz
  $0 verify-phase1
  $0 smoke
EOF
    exit 1
    ;;
esac
