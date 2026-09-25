#!/bin/bash
# 一键修复所有容器依赖 (每次容器重启后执行)
# 挂载在 /workspace/LeggedManip_Lab/docker/ 下，容器可访问

set -e
source /isaac-sim/setup_python_env.sh
export PATH=/isaac-sim/kit/python/bin:$PATH
export GIT_PYTHON_REFRESH=quiet

echo "[setup] 安装 git..."
apt-get update -qq 2>/dev/null
apt-get install -y -qq git 2>/dev/null || true

echo "[setup] 修复 pip..."
python3 -m ensurepip --upgrade --default-pip 2>/dev/null || true

echo "[setup] 安装 Isaac Lab..."
if [ ! -d /tmp/IsaacLab ]; then
    cp -r /workspace/IsaacLab /tmp/IsaacLab
fi
cd /tmp/IsaacLab
ln -sf /isaac-sim _isaac_sim 2>/dev/null

# 主包 (不递归装依赖，全在 Isaac Sim 里)
for pkg in isaaclab isaaclab_assets isaaclab_mimic isaaclab_rl isaaclab_tasks; do
    python3 -m pip install -q --no-build-isolation --no-deps -e source/$pkg 2>/dev/null || true
done

echo "[setup] 安装额外依赖..."
python3 -m pip install -q --no-deps flatdict gymnasium h5py 2>/dev/null || true

echo "[setup] 安装 rsl-rl-lib..."
python3 -m pip install -q rsl-rl-lib 2>/dev/null || true

echo "[setup] 修复 GitPython 依赖..."
python3 -m pip install -q --no-deps gitdb smmap GitPython 2>/dev/null || true

echo "[setup] 安装 LeggedManip Lab..."
rm -rf /tmp/LeggedManip_Lab 2>/dev/null
cp -r /workspace/LeggedManip_Lab/source/LeggedManip_Lab /tmp/LeggedManip_Lab
python3 -m pip install -q --no-build-isolation --no-deps -e /tmp/LeggedManip_Lab 2>/dev/null || true

echo "[setup] 配置环境变量..."
export ISAAC_PATH=/isaac-sim
export EXP_PATH=/isaac-sim/apps
export CARB_APP_PATH=/isaac-sim/kit
export GIT_PYTHON_REFRESH=quiet

echo "[setup] ✅ 完成!"
python3 -c "import isaaclab; print(f'  Isaac Lab {isaaclab.__version__}')" 2>/dev/null || echo "  ⚠ isaaclab 导入失败"
python3 -c "import rsl_rl; print('  rsl-rl OK')" 2>/dev/null || echo "  ⚠ rsl-rl 导入失败"
python3 -c "import gymnasium; print('  gymnasium OK')" 2>/dev/null || echo "  ⚠ gymnasium 导入失败"
which git && echo "  git OK" || echo "  ⚠ git 不可用"
