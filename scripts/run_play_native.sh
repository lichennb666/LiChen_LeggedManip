#!/usr/bin/env bash
# 一键 play/eval LeggedManip_Lab（自动切环境、自动定位最新 run/checkpoint）。
#
# 用法:
#   ~/LeggedManip_Lab/scripts/run_play_native.sh                    # 最新 run 的最新 checkpoint，带界面
#   ~/LeggedManip_Lab/scripts/run_play_native.sh <checkpoint.pt>    # 指定 checkpoint
#   HEADLESS=1 VIDEO=1 ~/LeggedManip_Lab/scripts/run_play_native.sh # 无界面，录一段视频后退出
#
# 可调环境变量:
#   LEGGEDMANIP_ENV / CONDA_SH / TASK / NUM_ENVS
#   RUN_DIR  (指定 run 目录)  CHECKPOINT (指定 .pt)
#   HEADLESS=1 / VIDEO=1 / VIDEO_LENGTH=400
#   DRYRUN=1 (只打印命令，不启动)
set -euo pipefail

ENV_NAME="${LEGGEDMANIP_ENV:-leggedmanip}"
CONDA_SH="${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
[ -f "$CONDA_SH" ] || { echo "找不到 conda.sh: $CONDA_SH" >&2; exit 1; }
# shellcheck disable=SC1090
source "$CONDA_SH"
conda activate "$ENV_NAME"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

TASK="${TASK:-GO2-PIPER-VBC-Teacher-Shape}"
NUM_ENVS="${NUM_ENVS:-1}"

# 指定 checkpoint：参数优先，其次 CHECKPOINT，否则取最新 run 的最新 model_*.pt
CKPT="${1:-}"
if [ -n "$CKPT" ] && [[ "$CKPT" == *.pt ]]; then
    shift || true
else
    CKPT="${CHECKPOINT:-}"
fi

RUN_DIR="${RUN_DIR:-}"
if [ -z "$RUN_DIR" ]; then
    RUN_DIR="$(ls -td "$PROJECT_ROOT"/logs/rsl_rl/*/*/ 2>/dev/null | grep -v '/exported/' | head -1 || true)"
fi
if [ -z "$CKPT" ] && [ -n "$RUN_DIR" ]; then
    CKPT="$(ls -t "$RUN_DIR"model_*.pt 2>/dev/null | head -1 || true)"
fi
if [ -z "$CKPT" ] || [ ! -f "$CKPT" ]; then
    echo "找不到 checkpoint。请用 CHECKPOINT=/path/model_x.pt 指定。" >&2
    echo "可用 run:"; ls -td "$PROJECT_ROOT"/logs/rsl_rl/*/*/ 2>/dev/null | head -5 >&2
    exit 1
fi

ARGS=(--task "$TASK" --checkpoint "$CKPT" --num_envs "$NUM_ENVS")
[ "${HEADLESS:-0}" = "1" ] && ARGS+=(--headless)
[ "${VIDEO:-0}" = "1" ] && ARGS+=(--video --video_length "${VIDEO_LENGTH:-400}")

export ACCEPT_EULA="${ACCEPT_EULA:-Y}"
export TERM="${TERM:-xterm}"

echo "[run_play_native] env=$ENV_NAME task=$TASK num_envs=$NUM_ENVS"
echo "[run_play_native] checkpoint=$CKPT"
echo "[run_play_native] headless=${HEADLESS:-0} video=${VIDEO:-0}"

if [ "${DRYRUN:-0}" = "1" ]; then
    echo "DRYRUN: python scripts/rsl_rl/play.py ${ARGS[*]} $*"
    exit 0
fi

exec python scripts/rsl_rl/play.py "${ARGS[@]}" "$@"
