#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The default demo is intentionally the minimal open-source-style manual
# sim2sim loop.  The former perception/mission demo remains available by
# explicitly setting LEGACY_MISSION=true.
if [[ "${LEGACY_MISSION:-false}" != "true" ]]; then
  exec "$script_dir/run_teleop_test.sh"
fi

gui="${GUI:-true}"
docker compose -f "$script_dir/compose.yaml" run --rm go2-piper-humble \
  ros2 launch go2_piper_bringup physics_mission.launch.py gui:="$gui" \
  physical_grasp:="${PHYSICAL_GRASP:-true}" \
  assisted_grasp:="${ASSISTED_GRASP:-true}"
