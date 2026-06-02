#!/usr/bin/env bash
# Clone third-party ROS 2 sources into ros2_ws/src/vendor/
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_SRC="${ROOT}/ros2_ws/src"
VENDOR_DIR="${WS_SRC}/vendor"

mkdir -p "${VENDOR_DIR}"

if ! command -v vcs >/dev/null 2>&1; then
  echo "ERROR: vcs (vcstool) not found. Install: apt install python3-vcstool"
  exit 1
fi

echo "==> Importing vendor repos into ${VENDOR_DIR}"
vcs import "${WS_SRC}" < "${ROOT}/repos/vendor.repos"

echo "==> Done. Next: ./scripts/bootstrap_workspace.sh"
