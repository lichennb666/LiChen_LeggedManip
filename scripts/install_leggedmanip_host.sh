#!/usr/bin/env bash
# =============================================================================
# LeggedManip_Lab 主机环境检测 + 安装脚本
#
# 用法:
#   ./install_leggedmanip_host.sh check     # 只检测主机环境并给出报告（默认）
#   ./install_leggedmanip_host.sh install   # 检测通过后执行安装
#   ./install_leggedmanip_host.sh all       # check + install + verify
#
# 可覆盖的环境变量:
#   LEGGEDMANIP_ENV    conda 环境名        (默认 leggedmanip)
#   ISAACLAB_DIR       IsaacLab 源码路径   (默认 ~/Project/IsaacLab)
#   LEGGEDMANIP_DIR    本仓库路径          (默认脚本上一级)
#   CONDA_SH           conda.sh 路径       (默认 ~/miniconda3/etc/profile.d/conda.sh)
#   ISAACSIM_VERSION   Isaac Sim 版本      (默认 5.1.0)
#   RSL_RL_VERSION     rsl-rl-lib 版本     (默认 5.0.1)
#   ISAACLAB_GIT       IsaacLab 仓库地址
# =============================================================================
set -uo pipefail

ACTION="${1:-check}"
ENV_NAME="${LEGGEDMANIP_ENV:-leggedmanip}"
ISAACLAB_DIR="${ISAACLAB_DIR:-$HOME/Project/IsaacLab}"
LEGGEDMANIP_DIR="${LEGGEDMANIP_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CONDA_SH="${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
ISAACSIM_VERSION="${ISAACSIM_VERSION:-5.1.0}"
RSL_RL_VERSION="${RSL_RL_VERSION:-5.0.1}"
ISAACLAB_GIT="${ISAACLAB_GIT:-https://github.com/isaac-sim/IsaacLab.git}"

# ---- output helpers ---------------------------------------------------------
if [ -t 1 ]; then C_G=$'\033[32m'; C_Y=$'\033[33m'; C_R=$'\033[31m'; C_B=$'\033[36m'; C_0=$'\033[0m'; else C_G=; C_Y=; C_R=; C_B=; C_0=; fi
ok()   { printf "%s[ OK ]%s %s\n" "$C_G" "$C_0" "$*"; }
warn() { printf "%s[WARN]%s %s\n" "$C_Y" "$C_0" "$*"; }
err()  { printf "%s[FAIL]%s %s\n" "$C_R" "$C_0" "$*"; }
info() { printf "%s[INFO]%s %s\n" "$C_B" "$C_0" "$*"; }
have() { command -v "$1" >/dev/null 2>&1; }

FAILED=0
fail() { err "$*"; FAILED=$((FAILED + 1)); }

# ---- checks -----------------------------------------------------------------
check_os() {
    info "OS / 架构"
    if [ ! -r /etc/os-release ]; then fail "无法读取 /etc/os-release"; return; fi
    # shellcheck disable=SC1091
    . /etc/os-release
    local arch; arch="$(uname -m)"
    printf "        %s %s (%s)\n" "${PRETTY_NAME:-unknown}" "${VERSION_ID:-}" "$arch"
    if [ "${ID:-}" != "ubuntu" ]; then fail "需要 Ubuntu（当前 ID=${ID:-?}）"; return; fi
    if [ "${VERSION_ID:-}" != "22.04" ]; then warn "推荐 Ubuntu 22.04（当前 ${VERSION_ID}）；其它版本未验证"; fi
    if [ "$arch" != "x86_64" ]; then fail "Isaac Sim 需要 x86_64（当前 $arch）"; fi
    ok "操作系统可用"
}

check_gpu() {
    info "NVIDIA GPU / 驱动"
    if ! have nvidia-smi; then fail "未找到 nvidia-smi：需要安装 NVIDIA 驱动"; return; fi
    local line; line="$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | head -1)"
    if [ -z "$line" ]; then fail "nvidia-smi 无输出（驱动异常）"; return; fi
    printf "        %s\n" "$line"
    local driver; driver="$(echo "$line" | awk -F', ' '{print $2}')"
    local major="${driver%%.*}"
    if [ -z "${major:-}" ] || ! [ "$major" -ge 0 ] 2>/dev/null; then warn "无法解析驱动版本：$driver"; return; fi
    if [ "$major" -lt 580 ]; then
        fail "驱动 $driver 太旧：Isaac Sim $ISAACSIM_VERSION 需要 >= 580"
    elif [ "$major" -eq 595 ]; then
        fail "驱动 595.x 与 Isaac Sim $ISAACSIM_VERSION 不兼容：请降到 580.x 或升级 Isaac Sim 6.0"
    else
        ok "驱动 $driver 可用"
    fi
    # 显存
    local mem; mem="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -1)"
    if [ -n "$mem" ] && [ "$mem" -lt 12000 ]; then warn "显存 ${mem}MiB 偏小：4096 环境训练建议 >=16GB"; fi
}

check_conda() {
    info "conda / Python"
    if [ ! -f "$CONDA_SH" ]; then
        if have conda; then CONDA_SH="$(conda info --base)/etc/profile.d/conda.sh"; else fail "未找到 conda（$CONDA_SH）"; return; fi
    fi
    # shellcheck disable=SC1090
    . "$CONDA_SH"
    printf "        conda: %s\n" "$(conda --version 2>/dev/null)"
    if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
        local pyv; pyv="$(conda run -n "$ENV_NAME" python --version 2>/dev/null)"
        printf "        env '%s' 已存在: %s\n" "$ENV_NAME" "$pyv"
        case "$pyv" in *"3.11"*) ok "Python 3.11" ;; *) warn "建议 Python 3.11（当前 $pyv）" ;; esac
    else
        warn "conda 环境 '$ENV_NAME' 不存在（install 会创建 python=3.11）"
    fi
}

check_proxy() {
    info "代理 / 网络"
    local px="${HTTP_PROXY:-${http_proxy:-}}"
    if [ -n "$px" ]; then
        warn "检测到代理 $px：Isaac 资产下载可能被劫持（见文档 §5）"
        if echo "${no_proxy:-}" | grep -q "amazonaws.com"; then
            ok "no_proxy 已包含 amazonaws.com"
        else
            warn "建议把 omniverse-content-production.s3-us-west-2.amazonaws.com 加入 no_proxy，或跑 Isaac 前 unset 代理"
        fi
    else
        ok "未设置代理"
    fi
    if have curl; then
        # 直连探测（坏代理会劫持 Isaac 资产，所以 --noproxy 更接近真实可用性）
        local code; code="$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' --max-time 15 \
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/" 2>/dev/null || true)"
        case "$code" in
            200|403) ok "Isaac 资产 S3 可直连 (HTTP $code)" ;;
            000) warn "Isaac 资产 S3 直连失败：检查网络；Isaac 首次运行会下资产，必须先通" ;;
            *)   ok "Isaac 资产 S3 可达 (HTTP $code)" ;;
        esac
        local pypi; pypi="$(curl -s -o /dev/null -w '%{http_code}' --max-time 12 https://pypi.org/simple/ 2>/dev/null || true)"
        [ "$pypi" = "200" ] && ok "PyPI 可达 (200)" || warn "PyPI 返回 $pypi（pip 可能需要换镜像）"
    else
        warn "未安装 curl，跳过网络探测"
    fi
}

check_disk() {
    info "磁盘空间"
    local avail; avail="$(df -Pk "$HOME" | awk 'NR==2{print int($4/1024/1024)}')"
    printf "        %s 可用: %s GB\n" "$HOME" "${avail:-?}"
    if [ -n "${avail:-}" ] && [ "$avail" -lt 40 ]; then warn "可用 <40GB：Isaac Sim 资产+环境建议预留 >=60GB"; else ok "空间充足"; fi
}

check_isaaclab() {
    info "IsaacLab 源码 / 工具链"
    if [ -d "$ISAACLAB_DIR/source/isaaclab" ]; then
        local ver; ver="$(cat "$ISAACLAB_DIR/VERSION" 2>/dev/null)"
        printf "        %s (VERSION=%s)\n" "$ISAACLAB_DIR" "${ver:-?}"
        local br=""
        if git -C "$ISAACLAB_DIR" rev-parse --abbrev-ref HEAD >/dev/null 2>&1; then
            br="$(git -C "$ISAACLAB_DIR" rev-parse --abbrev-ref HEAD)"
        fi
        if [ "$br" = "main" ]; then
            ok "IsaacLab 在 main 分支（VERSION 文件值不影响）"
        elif [ -n "$br" ]; then
            warn "IsaacLab 分支为 '$br'（建议 main；v2.3.2 tag 与本仓库不兼容）"
        else
            warn "IsaacLab 不是 git 仓库，无法确认分支（需 main）"
        fi
    else
        warn "未找到 IsaacLab：$ISAACLAB_DIR（install 会克隆 main）"
    fi
    have git && ok "git 已安装" || fail "缺少 git"
}

report() {
    echo "============================================================"
    echo " LeggedManip_Lab 主机环境检测报告"
    echo "============================================================"
    check_os; echo
    check_gpu; echo
    check_conda; echo
    check_isaaclab; echo
    check_disk; echo
    check_proxy; echo
    echo "============================================================"
    if [ "$FAILED" -eq 0 ]; then
        ok "检测通过（FAIL=0）。可执行: $0 install"
    else
        err "检测发现 $FAILED 个硬性问题，请先修复后再 install"
        return 1
    fi
}

# ---- install ----------------------------------------------------------------
install_env() {
    info "创建/复用 conda 环境: $ENV_NAME"
    # shellcheck disable=SC1090
    . "$CONDA_SH"
    if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
        conda create -n "$ENV_NAME" python=3.11 -y || return 1
    fi
    # shellcheck disable=SC1090
    conda activate "$ENV_NAME"
    python -m pip install --upgrade pip || return 1
    ok "环境就绪: $(python --version)"
}

install_isaacsim() {
    info "安装 Isaac Sim $ISAACSIM_VERSION + torch cu128"
    conda run -n "$ENV_NAME" python -m pip show isaacsim >/dev/null 2>&1 && \
        { ok "isaacsim 已安装，跳过"; return 0; }
    conda run -n "$ENV_NAME" python -m pip install "isaacsim[all,extscache]==$ISAACSIM_VERSION" || return 1
    conda run -n "$ENV_NAME" python -m pip install torch==2.7.0 torchvision==0.22.0 \
        --index-url https://download.pytorch.org/whl/cu128 || return 1
    ok "Isaac Sim 安装完成"
}

install_isaaclab() {
    info "准备 IsaacLab (main)"
    if [ ! -d "$ISAACLAB_DIR/source/isaaclab" ]; then
        git clone "$ISAACLAB_GIT" "$ISAACLAB_DIR" || return 1
    fi
    git -C "$ISAACLAB_DIR" fetch --all --tags >/dev/null 2>&1 || true
    git -C "$ISAACLAB_DIR" checkout main >/dev/null 2>&1 || warn "切换 main 失败，沿用当前分支"
    conda run -n "$ENV_NAME" python -m pip install \
        -e "$ISAACLAB_DIR/source/isaaclab" \
        -e "$ISAACLAB_DIR/source/isaaclab_assets" \
        -e "$ISAACLAB_DIR/source/isaaclab_mimic" \
        -e "$ISAACLAB_DIR/source/isaaclab_rl" \
        -e "$ISAACLAB_DIR/source/isaaclab_tasks" || return 1
    ok "IsaacLab 安装完成"
}

install_rsl_and_project() {
    info "安装 rsl-rl-lib 与 LeggedManip_Lab"
    conda run -n "$ENV_NAME" python -m pip install "rsl-rl-lib==$RSL_RL_VERSION" 'onnxscript>=0.5' || return 1
    conda run -n "$ENV_NAME" python -m pip install -e "$LEGGEDMANIP_DIR/source/LeggedManip_Lab" || return 1
    [ -f "$LEGGEDMANIP_DIR/requirements.txt" ] && \
        conda run -n "$ENV_NAME" python -m pip install -r "$LEGGEDMANIP_DIR/requirements.txt"
    # 对齐 isaacsim-kernel 的严格 pin
    conda run -n "$ENV_NAME" python -m pip install 'numpy==1.26.0' 'psutil==5.9.8' 'typing_extensions==4.12.2' || return 1
    ok "LeggedManip_Lab 安装完成"
}

verify() {
    info "验证安装"
    conda run -n "$ENV_NAME" python - <<'PY' || return 1
import importlib.metadata as md
import isaaclab, isaaclab_tasks, isaaclab_rl, rsl_rl  # noqa: F401
print("  isaacsim     :", md.version("isaacsim"))
print("  isaaclab     :", md.version("isaaclab"))
print("  rsl-rl-lib   :", md.version("rsl-rl-lib"))
print("  torch        :", md.version("torch"))
print("  numpy        :", md.version("numpy"))
v = md.version("rsl-rl-lib")
assert v >= "5.0.1", f"rsl-rl-lib {v} < 5.0.1 (装成了 v2.3.2 tag?)"
print("  imports OK")
PY
    ok "验证通过"
}

usage() { sed -n '2,20p' "$0"; }

main() {
    case "$ACTION" in
        check)  report ;;
        install) report || { err "检测未通过，终止安装"; exit 1; }; install_env && install_isaacsim && install_isaaclab && install_rsl_and_project && ok "安装完成" ;;
        all)    report || { err "检测未通过，终止安装"; exit 1; }; install_env && install_isaacsim && install_isaaclab && install_rsl_and_project && verify ;;
        -h|--help|help) usage ;;
        *) echo "未知参数: $ACTION" >&2; usage; exit 2 ;;
    esac
}
main
