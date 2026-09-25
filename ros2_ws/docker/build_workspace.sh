#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker compose -f "$script_dir/compose.yaml" run --rm go2-piper-humble \
  bash -lc 'source /opt/ros/humble/setup.bash && colcon build --symlink-install --event-handlers console_direct+'

