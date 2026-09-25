#!/usr/bin/env bash
# 带自动续训的训练启动器。
#
# Isaac Sim 偶发 native 崩溃（如 carb.tasking Mutex 断言）会让训练中途退出。
# 本脚本在训练非零退出时，自动从当前 run 最新的 model_*.pt 续训，直到正常结束
# 或达到最大重试次数。stage-2 解冻底盘也用它（配合 VBC_VELOCITY_SCALE）。
#
# 用法:
#   TASK=GO2-PIPER-VBC-Teacher-Shape NUM_ENVS=4096 MAX_ITERS=10000 \
#     ~/LeggedManip_Lab/scripts/run_train_watchdog.sh
#
#   # stage 2 解冻底盘、从阶段①续训（首次即 resume）:
#   VBC_VELOCITY_SCALE=0.4,0.3,0.5 RESUME_RUN=<阶段①run名> RESUME_CKPT=model_5000.pt \
#   TASK=GO2-PIPER-VBC-Teacher-Shape NUM_ENVS=4096 MAX_ITERS=20000 \
#     ~/LeggedManip_Lab/scripts/run_train_watchdog.sh
#
# 可调: LEGGEDMANIP_ENV / CONDA_SH / TASK / NUM_ENVS / MAX_ITERS / MAX_RETRIES
#       RESUME_RUN / RESUME_CKPT (强制首次即 resume)
#       VBC_VELOCITY_SCALE (0,0,0=冻结底盘；非零=解冻)
set -uo pipefail

ENV_NAME="${LEGGEDMANIP_ENV:-leggedmanip}"
CONDA_SH="${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
[ -f "$CONDA_SH" ] || { echo "找不到 conda.sh: $CONDA_SH" >&2; exit 1; }
# shellcheck disable=SC1090
source "$CONDA_SH"
conda activate "$ENV_NAME"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

export ACCEPT_EULA="${ACCEPT_EULA:-Y}"
export TERM="${TERM:-xterm}"

TASK="${TASK:-GO2-PIPER-VBC-Teacher-Shape}"
NUM_ENVS="${NUM_ENVS:-4096}"
MAX_ITERS="${MAX_ITERS:-10000}"
MAX_RETRIES="${MAX_RETRIES:-30}"
LOG_DIR="${LOG_DIR:-$PROJECT_ROOT/logs/console}"
mkdir -p "$LOG_DIR"

exp_of_task() {
    case "$1" in
        *Shape*Teacher*)      echo go2_piper_vbc_shape_teacher ;;
        *Teacher*)            echo go2_piper_vbc_teacher ;;
        *MaskDepth*Student*)  echo go2_piper_vbc_mask_depth_student ;;
        *Student*)            echo go2_piper_vbc_student ;;
        *WBC*)                echo go2_piper_wbc ;;
        *)                    echo "" ;;
    esac
}
EXP="$(exp_of_task "$TASK")"

RESUME_RUN="${RESUME_RUN:-}"
RESUME_CKPT="${RESUME_CKPT:-}"

echo "[watchdog] env=$ENV_NAME task=$TASK num_envs=$NUM_ENVS max_iterations=$MAX_ITERS"
echo "[watchdog] VBC_VELOCITY_SCALE=${VBC_VELOCITY_SCALE:-<unset → 0,0,0 冻结底盘>}"

for attempt in $(seq 1 "$MAX_RETRIES"); do
    ARGS=(--task "$TASK" --num_envs "$NUM_ENVS" --headless --max_iterations "$MAX_ITERS")

    if [ -n "$RESUME_RUN" ] && [ -n "$RESUME_CKPT" ]; then
        ARGS+=(--resume --load_run "$RESUME_RUN" --checkpoint "$RESUME_CKPT")
    elif [ "$attempt" -gt 1 ] && [ -n "$EXP" ]; then
        RUN_DIR="$(ls -td "$PROJECT_ROOT"/logs/rsl_rl/"$EXP"/*/ 2>/dev/null | grep -v '/exported/' | head -1 || true)"
        if [ -n "$RUN_DIR" ]; then
            CKPT="$(ls -t "$RUN_DIR"model_*.pt 2>/dev/null | head -1 || true)"
            if [ -n "$CKPT" ]; then
                ARGS+=(--resume --load_run "$(basename "$RUN_DIR")" --checkpoint "$(basename "$CKPT")")
                echo "[watchdog] 将从 $RUN_DIR$(basename "$CKPT") 续训"
            fi
        fi
    fi

    TS="$(date +%Y%m%d_%H%M%S)"
    LOG="$LOG_DIR/${TASK}_${TS}.log"
    echo "[watchdog] ===== 第 $attempt/$MAX_RETRIES 次启动，日志：$LOG ====="
    set +e
    python scripts/rsl_rl/train.py "${ARGS[@]}" "$@" 2>&1 | tee "$LOG"
    rc=${PIPESTATUS[0]}
    set -e
    if [ "$rc" -eq 0 ]; then
        echo "[watchdog] 训练正常结束 (rc=0)"
        exit 0
    fi
    echo "[watchdog] 训练异常退出 (rc=$rc)，10 秒后自动续训…"
    # 强制后续 attempt 走 resume
    RESUME_RUN=""
    RESUME_CKPT=""
    sleep 10
done

echo "[watchdog] 达到最大重试次数 $MAX_RETRIES，放弃。最新日志在 $LOG_DIR" >&2
exit 1
