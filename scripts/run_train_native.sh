#!/usr/bin/env bash
# 一键启动 LeggedManip_Lab 的 IsaacLab 训练（原生 conda 环境）。
#
# 用法:
#   ~/LeggedManip_Lab/scripts/run_train_native.sh
#   TASK=GO2-PIPER-VBC-Student-MaskDepth NUM_ENVS=32 \
#     ~/LeggedManip_Lab/scripts/run_train_native.sh --enable_cameras
#
# 可调环境变量:
#   LEGGEDMANIP_ENV (默认 leggedmanip) / TASK / NUM_ENVS / MAX_ITERS / CONDA_SH
# 额外命令行参数会原样追加给 train.py。
#
# 说明: leggedmanip 环境已带自动“代理例外”钩子，无需手动 unset 代理。
set -euo pipefail

ENV_NAME="${LEGGEDMANIP_ENV:-leggedmanip}"
CONDA_SH="${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
if [ ! -f "${CONDA_SH}" ]; then
    echo "找不到 conda.sh: ${CONDA_SH}，请设置 CONDA_SH 指向你的 conda" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "${CONDA_SH}"
conda activate "${ENV_NAME}"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

export ACCEPT_EULA="${ACCEPT_EULA:-Y}"
export TERM="${TERM:-xterm}"

TASK="${TASK:-GO2-PIPER-VBC-Teacher-Shape}"
NUM_ENVS="${NUM_ENVS:-512}"
MAX_ITERS="${MAX_ITERS:-10000}"

echo "[run_train_native] env=${ENV_NAME} task=${TASK} num_envs=${NUM_ENVS} max_iterations=${MAX_ITERS}"
exec python scripts/rsl_rl/train.py \
    --task "${TASK}" \
    --num_envs "${NUM_ENVS}" \
    --headless \
    --max_iterations "${MAX_ITERS}" \
    "$@"
