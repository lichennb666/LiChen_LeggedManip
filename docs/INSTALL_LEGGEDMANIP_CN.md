# LeggedManip_Lab 安装文档（新机器）

IsaacLab 强化学习训练 + MuJoCo 部署工程。面向 **Ubuntu 22.04 + NVIDIA GPU**，
按本机实测通过的最小流程整理。

---

## 0. 环境要求（版本必须匹配）

| 组件 | 版本 | 说明 |
|---|---|---|
| OS | Ubuntu 22.04 x86_64 | |
| NVIDIA 驱动 | **580.x** | Isaac Sim 5.1 要求；595 不兼容 |
| Python | **3.11**（conda） | |
| Isaac Sim | **5.1.0** | pip 安装 |
| Isaac Lab | **main 分支** | 不能用 `v2.3.2` tag（它 pin `rsl-rl-lib==3.1.2`，与本仓库不兼容） |
| rsl-rl-lib | **>=5.0.1** | |
| torch | **2.7.0+cu128** | |

> 本仓库代码面向 Isaac Lab `main` 的新 RSL-RL 配置 API。

---

## 1. 创建 conda 环境

```bash
conda create -n leggedmanip python=3.11 -y
conda activate leggedmanip
python -m pip install --upgrade pip
```

---

## 2. 安装 Isaac Sim 5.1 + PyTorch

```bash
# Isaac Sim（pip 路线，含全部扩展）
pip install 'isaacsim[all,extscache]==5.1.0'

# PyTorch cu128
pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

> 首次运行会要求接受 NVIDIA EULA：`export ACCEPT_EULA=Y`。

---

## 3. 安装 Isaac Lab（main 分支，源码 editable）

```bash
git clone https://github.com/isaac-sim/IsaacLab.git ~/Project/IsaacLab
cd ~/Project/IsaacLab
git checkout main
pip install -e source/isaaclab \
            -e source/isaaclab_assets \
            -e source/isaaclab_mimic \
            -e source/isaaclab_rl \
            -e source/isaaclab_tasks
```

> Isaac Sim 5.1 的 pip 包与 IsaacLab main 需一起安装；`isaaclab_rl` 的 rsl-rl extra
> 在本仓库用 `main` 时对应 `rsl-rl-lib==5.0.1`。

---

## 4. 安装 rsl-rl 与 LeggedManip_Lab

```bash
# rsl-rl（与 IsaacLab main 匹配的版本）
pip install 'rsl-rl-lib==5.0.1' 'onnxscript>=0.5'

# 本仓库（注意：克隆到 Isaac Lab 目录之外）
pip install -e ~/LeggedManip_Lab/source/LeggedManip_Lab
pip install -r ~/LeggedManip_Lab/requirements.txt

# 对齐 isaacsim-kernel 的严格 pin（否则运行时可能报版本不符）
pip install 'numpy==1.26.0' 'psutil==5.9.8' 'typing_extensions==4.12.2'
```

验证：

```bash
python -m pip show rsl-rl-lib | grep Version     # 应为 5.0.1 或更高
python -m pip show isaacsim | grep Version        # 5.1.0
```

---

## 5. 网络 / 代理（重要）

Isaac Sim 的默认场景资产从 NVIDIA S3 下载。如果本机设了**坏代理**，会报：

```
Unable to open the usd file at path:
https://omniverse-content-production.s3-us-west-2.amazonaws.com/.../default_environment.usd
```

两种处理：

```bash
# 方式 1：跑 Isaac 前临时清掉代理
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy

# 方式 2：只让 Isaac 域名绕过代理（其它下载仍走代理）
export no_proxy="omniverse-content-production.s3-us-west-2.amazonaws.com,$no_proxy"
```

（本机做法：在 conda 环境 `etc/conda/activate.d/` 里加脚本，激活 `leggedmanip` 时只追加
`no_proxy`，`conda deactivate` 恢复。）

---

## 6. 生成 VBC 所需的 URDF / 物体资产

`GO2-PIPER-VBC-*` 系列任务使用从 Gazebo URDF 转换来的 `go2_piper_vbc.urdf`
（含可驱动的 `joint7/joint8` 夹爪）。若工程自带的 `assets/go2_piper/` 下没有该文件：

```bash
python ~/LeggedManip_Lab/scripts/prepare_go2_piper_vbc_urdf.py \
  --input  <Go2Arm_sim2sim>/ros2_ws/src/go2_piper_description/urdf/go2_piper_ros2.urdf \
  --output ~/LeggedManip_Lab/source/LeggedManip_Lab/LeggedManip_Lab/assets/go2_piper/go2_piper_vbc.urdf \
  --mesh-root <Go2Arm_sim2sim>/ros2_ws/src/go2_piper_description/legacy/meshes
```

物体资产（PointNet++ 特征 + 网格）若缺失：

```bash
python ~/LeggedManip_Lab/scripts/prepare_vbc_object_assets.py
```

> 注意：这一步依赖 `Go2Arm_sim2sim` 里的 URDF/网格。若只部署本仓库，需把对应 URDF 与
> `legacy/meshes` 一并带过来。

---

## 7. 首次冒烟测试

```bash
export ACCEPT_EULA=Y TERM=xterm
conda activate leggedmanip
cd ~/LeggedManip_Lab

# 4 个环境跑 1 次迭代
python scripts/rsl_rl/train.py --task GO2-PIPER-VBC-Teacher-Shape \
  --num_envs 4 --headless --max_iterations 1
```

出现 `Learning iteration 0/1 ... Training time` 即成功。

---

## 8. 训练 / 部署命令

### 8.1 WBC（底层全身控制器）
```bash
TASK=GO2-PIPER-WBC NUM_ENVS=4096 MAX_ITERS=8000 \
  ~/LeggedManip_Lab/scripts/run_train_watchdog.sh
# 从已有 checkpoint 续训：
#   ... run_train_watchdog.sh --resume --load_run <run> --checkpoint model_<N>.pt
```

### 8.2 VBC teacher（高层，用冻结 WBC）
```bash
LEGGEDMANIP_WBC_POLICY=<wbc-run>/exported/policy.pt \
VBC_VELOCITY_SCALE=0,0,0 \
TASK=GO2-PIPER-VBC-Teacher-Shape NUM_ENVS=4096 MAX_ITERS=10000 \
  ~/LeggedManip_Lab/scripts/run_train_watchdog.sh
```

### 8.3 VBC 视觉 student（蒸馏）
```bash
TASK=GO2-PIPER-VBC-Student-MaskDepth NUM_ENVS=32 MAX_ITERS=10000 \
  ~/LeggedManip_Lab/scripts/run_train_watchdog.sh \
  --enable_cameras --teacher_checkpoint <teacher>/model_<N>.pt
```

### 8.4 play / 看曲线 / 诊断
```bash
# GUI play
LEGGEDMANIP_WBC_POLICY=<wbc>/exported/policy.pt \
python scripts/rsl_rl/play.py --task GO2-PIPER-VBC-Teacher-Shape \
  --checkpoint logs/rsl_rl/go2_piper_vbc_shape_teacher/<run>/model_<N>.pt --num_envs 1

# 曲线
python scripts/plot_metrics.py --exp go2_piper_vbc_shape_teacher --out ~/vbc.png

# 诊断：末端跟踪误差 / 物体抬升高度
LEGGEDMANIP_WBC_POLICY=<wbc>/exported/policy.pt python scripts/rsl_rl/vbc_diagnose.py \
  --task GO2-PIPER-VBC-Teacher-Shape --checkpoint <ckpt> --num_envs 64 --steps 600 --headless
```

环境变量：
- `LEGGEDMANIP_WBC_POLICY`：VBC 使用的冻结 WBC（不设则用默认）
- `VBC_VELOCITY_SCALE`：`0,0,0` 冻结底盘 / `0.4,0.3,0.5` 解冻
- `LEGGEDMANIP_ENV`（默认 `leggedmanip`）、`CONDA_SH`

---

## 9. MuJoCo 部署

MuJoCo 只负责部署/回放策略，不能替代 IsaacLab 训练。建议独立 venv：

```bash
python3 -m venv --system-site-packages ~/Go2Arm_sim2sim/.venv-leggedmanip
~/Go2Arm_sim2sim/.venv-leggedmanip/bin/python -m pip install -r ~/LeggedManip_Lab/requirements.txt

~/Go2Arm_sim2sim/.venv-leggedmanip/bin/python \
  ~/LeggedManip_Lab/mujoco/deploy/deploy_mujoco/go2_piper/go2_piper.py config_wbc.yaml
```

（键盘：WASDQE 底盘、IJKLUO 末端、1~6 姿态、R 急停、ESC 退出。
`config_wbc_tuned.yaml` 指向微调后的 WBC。）

---

## 10. 本项目专属已知坑

### 10.1 底层 WBC 的命令范围与精度（关键）
- 原 WBC 训练时 EE 位置命令范围极小：`x(0.40,0.45) / y(±0.05) / z=0.5`；VBC 场景的物体在
  x≈0.62，需放宽 `wbc_env_cfg.py` 的 `pos_x/pos_y/pos_z` 才能覆盖。
- 跟踪奖励核 `std≈0.316` 太软 → 精度只有 ~6.6cm；抓 6cm 物体需要 **<3cm**，
  应把 `std=0.06`、位置 `weight=6.0`、姿态 `weight=-8.0` 并精修。
- VBC 的 EE 命令盒不要包含**桌面以下/不可达**区域（否则 WBC 永远追不上）。

### 10.2 VBC 奖励易陷局部最优
靠近奖励过大时会停在"碰一下"。建议：`object_reach≈3`、`gripper_contact≈3`、
`object_lift_progress≈10`（分段：<3cm 不给分、10cm 给满）、`physical_grasp_success≈40`，
并删除自指的 EE 命令跟踪奖励。

### 10.3 `save_interval`
WBC 默认 1000，中断（kill）会丢掉最近的改进。建议改小到 **250**。

### 10.4 task 名字
- 训练：`GO2-PIPER-WBC`、`GO2-PIPER-VBC-Teacher-Shape`、`GO2-PIPER-VBC-Student-MaskDepth`
- play 变体：`GO2-PIPER-WBC-Play`（VBC Shape 没有 Play 变体，直接用本体）
- play / VBC 训练务必带 `LEGGEDMANIP_WBC_POLICY`。

### 10.5 显存
一个 4096 的 Isaac 训练约占 **7~12GB**。24GB 卡最多并发 2 个；第三个会 OOM 或把迭代拖慢数倍。

### 10.6 别用 IsaacLab `v2.3.2` tag
它 pin `rsl-rl-lib==3.1.2`，与本仓库 agent 配置不兼容（应显示 >=5.0.1）。

---

## 11. 验收清单

```bash
conda activate leggedmanip
python -c "import isaaclab, isaaclab_tasks, rsl_rl; print('imports OK')"
python -m pip show rsl-rl-lib | grep Version
export ACCEPT_EULA=Y
python scripts/rsl_rl/train.py --task GO2-PIPER-VBC-Teacher-Shape \
  --num_envs 4 --headless --max_iterations 1     # 冒烟
```
