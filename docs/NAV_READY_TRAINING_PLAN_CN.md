# Gazebo 导航就绪训练计划（第二阶段）

> 目标：让现有 GO2-PIPER-WBC 策略在 Gazebo（ODE）域内具备可靠的
> “速度指令跟踪 + 站位保持 + 臂扰动下不漂移”能力，为接入 Nav2/航点导航
> 以及最终 Sim-to-Real 打底。第一阶段（部署端）已完成：cmd_vel 指令适配层、
> 臂阻尼、抓取前向偏置等；本文件只覆盖训练侧改动。

## 1. 问题回顾（为什么要改训练）

实测证据（2026-08-07）：

- 策略在 Gazebo 里能站（10s 站立门禁通过）、能走（0.4 m/s 指令 4s 前进 0.19m）；
- 但低速平移指令跟踪差（0.03 m/s 级别指令表现为转向而非平移）；
- 零指令站位下会随机漂移，臂运动（IK 伺服）时漂移放大到 0.1~0.9m，方向随机；
- 策略输出在 50Hz 下有高频尖峰（单步目标最大 0.67 rad），说明反馈增益按 Isaac
  plant 调校，迁移到 ODE 后闭环振荡。

根因：训练域（Isaac PhysX）与 Gazebo ODE 的接触/摩擦/关节动力学差异超出当前
域随机化覆盖范围；且训练奖励以“速度跟踪”为主，没有“位置保持”项——零速度指令下
速度误差为零但位置持续漂移，策略认为任务已完成。

## 2. 验收门禁（训练完成后在 Gazebo 无头环境逐项跑）

每次验收跑 N=5 轮，全部通过才算达标（当前结果逐轮随机，必须统计意义）。

| # | 门禁 | 场景 | 通过标准 |
|---|------|------|----------|
| G1 | 零指令站位保持 | 站立 30s，无指令 | 底盘水平位移 < 0.05m |
| G2 | 臂扰动站位保持 | 站立 + ee_pose 随机指令 30s | 底盘水平位移 < 0.10m，臂跟踪误差 < 0.05m |
| G3 | 低速跟踪 | 0.1 m/s 直线 10s | 位置误差 < 0.05m |
| G4 | 中速跟踪 | 0.5 m/s 直线 10s | 位置误差 < 0.10m |
| G5 | 停车精度 | 0.5 m/s 后指令归零 | 1s 内停住，最终位置误差 < 0.05m |
| G6 | 运输保持 | 抓取 + 携带 0.5m | mission 的 `PHYSICAL_TRANSPORT_FAILED` 不再触发（offset < 0.09m） |
| G7 | 导航闭环 | Nav2 或航点跟随 3 点路线 | 到达误差 < 0.10m，无振荡超时 |

## 3. 域随机化改动（按优先级）

### 3.1 执行器延迟与抖动（最高优先级）

训练已含 0~4 tick 控制延迟随机化，Gazebo 实测仍需更宽覆盖。在
`assets/go2_piper/go2_piper_articulation_cfg.py` 的
`DelayedPDActuatorCfg` 中：

```python
joint_delay: dict[str, tuple[float, float]] = {".*": (0.0, 0.08)}  # 0-4 tick @50Hz
# 增加执行器增益随机化幅度（当前 ±20% → ±40%）
```

同时验证 `randomize_actuator_gains` 的
`actuator_gain_distribution_params` 从 (0.8, 1.2) 改为 (0.6, 1.4)。

### 3.2 摩擦与接触（高优先级）

`leggedmanip_lab_env_cfg.py` 的 `EventCfg`：

```python
randomize_friction = EventTerm(
    func=mdp.randomize_rigid_body_material,
    params={
        "asset_cfg": SceneEntityCfg("robot", body_names=[".*"]),
        "static_friction_range": (0.4, 1.5),   # 原 (0.5, 1.2)
        "dynamic_friction_range": (0.4, 1.2),  # 原 (0.5, 1.2)
        "restitution_range": (0.0, 0.15),
        "num_buckets": 128,                    # 更多桶减少跨环境耦合
    },
)
```

地面 `SceneCfg.plane` 摩擦改为 `(0.9, 0.9)` 附近并保留 combine multiply，
与 Gazebo 地面/足端接触参数的差异方向对齐。

### 3.3 站位保持奖励（本文件最关键的奖励改动）

在 `RewardsCfg` 新增两项，权重课程式推进：

```python
# 1) 速度指令≈0 时的基座水平位移惩罚（防止“零速度但持续漂移”）
base_pos_hold_l2 = RewTerm(
    func=mdp.base_position_l2_exp,
    weight=-2.0,
    params={"cmd_vel_deadband": 0.05},  # 仅当 |cmd| < 0.05 时激活
)

# 2) 低速指令的跟踪误差加权（低速精度比高速更难，给更高权重）
track_lin_vel_low_speed_exp = RewTerm(
    func=mdp.velocity_command_error_exp,
    weight=4.0,
    params={"low_speed_threshold": 0.2},  # 指令低于 0.2 m/s 时启用
)
```

`base_position_l2_exp` 的参考点取 episode 内最后一次指令 ≥ 阈值时刻的基座位置
（可在环境状态里维护 `hold_reference_pos`，指令归零时重置），避免与行走任务冲突。

### 3.4 命令课程（低速优先）

`wbc_env_cfg.py` 的 `WBCCommandsCfg.base_velocity`：

```python
# 初始范围从 ±0.2 降到 ±0.05，课程每过关一级 +0.05，到 ±1.0 为止；
# 让策略先学会精确的低速执行，再放开速度。
ranges=UniformVelocityCommandCfg.Ranges(
    lin_vel_x=(-0.05, 0.05),
    lin_vel_y=(-0.05, 0.05),
    ang_vel_z=(-0.05, 0.05),
),
curriculum_enabled=True,
```

### 3.5 基座高度与 IMU（中优先级）

- `track_base_height_exp` 目标高度随机化到 (0.26, 0.32)，覆盖 Gazebo 实测
  0.287m（当前训练固定 0.28/0.30）；
- 观测噪声 `base_ang_vel`、`projected_gravity` 的噪声幅度上调 30%，
  覆盖 Gazebo IMU 抖动（Phase 8 已定位 IMU 姿态/角速度是主要噪声源之一）；
- 增加 `randomize_rigid_body_inertia` 幅度 (0.8, 1.2) → (0.7, 1.3)。

## 4. 训练执行建议

1. **从现有 WBC checkpoint 微调而非从零训练**：保留
   `logs/rsl_rl/go2_piper_wbc/` 下最新模型，`rsl_rl_ppo_cfg.py` 里
   `load_run`/`load_checkpoint` 指向它，`learning_rate` 降到 3e-4；
2. 先只加“站位保持奖励 + 低速课程”跑一轮，确认 G1/G2/G3 通过，再叠加
   3.1/3.2 的随机化扩幅（一次只改一类变量，避免无法归因）；
3. 每轮训练后在 MuJoCo 回归（不能退步），再进 Gazebo 跑第 2 节门禁；
4. 门禁全过后接 Nav2：`nav2` → `/cmd_vel` → wbc 指令适配层（第一阶段已实现）
   → 策略，用 G7 闭环验收。

## 5. 相关文件索引

| 文件 | 改动点 |
|------|--------|
| `source/LeggedManip_Lab/.../go2_piper/wbc_env_cfg.py` | 命令范围/课程、奖励权重、PLAY 配置 |
| `source/LeggedManip_Lab/.../leggedmanip_lab_env_cfg.py` | EventCfg 随机化、RewardsCfg 新奖励、CurriculumCfg |
| `source/LeggedManip_Lab/.../assets/go2_piper/go2_piper_articulation_cfg.py` | DelayedPD 延迟/armature、执行器增益随机化 |
| `source/LeggedManip_Lab/.../agents/rsl_rl_ppo_cfg.py` | checkpoint 微调、学习率 |

> 注意：以上数值是建议起点，需在实际训练中按门禁结果迭代；训练配置修改需在
> Isaac Lab 训练环境中验证，不能仅靠 Gazebo 部署端反推。
