#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
gui="${GUI:-false}"
docker compose -f "$script_dir/compose.yaml" run --rm go2-piper-humble \
  ros2 launch go2_piper_bringup wbc_gate.launch.py gui:="$gui" "$@"
