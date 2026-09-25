#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
gui="${GUI:-true}"
launch_file="${LAUNCH_FILE:-teleop_test.launch.py}"
launch_container="go2-piper-teleop-launch"

if ! docker image inspect go2-piper-humble:latest >/dev/null 2>&1; then
  echo "ERROR: image go2-piper-humble:latest not found" >&2
  echo "       run ./docker/build_image.sh first" >&2
  exit 1
fi

echo "[teleop] starting Gazebo scene (${launch_file}, gui=${gui}) in background..."
docker compose -f "$script_dir/compose.yaml" run -d --name "$launch_container" -T go2-piper-humble \
  ros2 launch go2_piper_bringup "$launch_file" gui:="$gui"

cleanup() {
  echo
  echo "[teleop] stopping Gazebo launch..."
  docker rm -f "$launch_container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[teleop] waiting for WBC policy readiness (/go2_piper/wbc/status ready=true)..."
docker compose -f "$script_dir/compose.yaml" run --rm go2-piper-humble \
  bash -lc '
    source /opt/ros/humble/setup.bash
    if [ -f /workspace/LeggedManip_Lab/ros2_ws/install/setup.bash ]; then
      source /workspace/LeggedManip_Lab/ros2_ws/install/setup.bash
    fi
    deadline=$((SECONDS + 180))
    ready=false
    while [ "$ready" != true ] && [ $SECONDS -lt $deadline ]; do
      status=$(timeout 8 ros2 topic echo /go2_piper/wbc/status --once 2>/dev/null || true)
      if printf "%s" "$status" | grep -q "\"ready\": true"; then
        ready=true
      else
        sleep 2
      fi
    done
    if [ "$ready" != true ]; then
      echo "ERROR: WBC policy did not become ready within 180s" >&2
      echo "       check the scene logs: docker logs go2-piper-teleop-launch" >&2
      exit 1
    fi
    echo "[teleop] ready! W/S/A/D/Q/E dog, I/K/J/L/U/O arm, G grasp, F retry, R release"
    ros2 run go2_piper_bringup teleop_keyboard
  '
