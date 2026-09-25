# 调研发现：DART 策略适配训练

## 1. 部署侧结论（详见 Go2Arm_sim2sim 迁移计划）
- 同一 `policy.pt`：MuJoCo 73%/78%/54% 可用，Gazebo Classic ODE 部分可用，**Gazebo Sim DART 无法站立**
- DART 无法站立不是初始条件/参数问题（已试摩擦 2.0、spawn 0.26、惯性修复），是策略与 DART 动力学不兼容
- 需要训练侧增强鲁棒性

## 2. 现有训练域随机化（leggedmanip_lab_env_cfg.py EventCfg）
| 类别 | 现有范围 | DART 需覆盖 |
|------|---------|------------|
| 物理材质摩擦 | (0.4, 1.5)，64 桶 | 需扩到 (0.3, 2.5)（DART world 用 mu=2.0） |
| 执行器增益 | scale ×(0.6, 1.4) | 可考虑改 add（刚度 ±2.0、阻尼 ±0.5） |
| 基座质心偏移 | ±0.05 | 保持 |
| 质量 | ×(0.9, 1.1) | 保持 |
| 关节随机角度 | reset 时 | 保持 |
| 推搡 | 10-15s，间隔 | 可选启用 |
| 惯量 | ×(0.7, 1.3) | 保持 |
| 关节摩擦/armature | 无 | **需新增**（DART 关节动力学差异大） |
| 基座初始姿态扰动 | reset_base 有 | 可加大（落地站稳） |

## 3. finetune 机制（已实现并验证）
- `train.py --finetune_policy <policy.pt>`：从导出策略初始化 actor（strict=False 跳过 shape 不匹配）
- 已验证 8/8 权重拷贝成功（见 2026-08-07-gazebo 计划 Phase 9）
- **注意**：WBC 观测 210 维（×3 历史），若加观测项会破坏 shape，finetune 部分失效

## 4. 训练环境（复用已验证配置）
- isaac-sim 容器（`isaaclab_enter` / `isaaclab_train` 命令，见 .bashrc）
- rsl-rl 5.4.1 / isaaclab 0.54.4
- 历史 finetune 参考：GO2-PIPER-WBC 1024 envs 2000 iters，迭代 1.25s，总 44min，奖励 0.02→96→149
- 训练命令入口：`python scripts/rsl_rl/train.py --task GO2-PIPER-WBC ...`

## 5. 相关文件
| 文件 | 用途 |
|------|------|
| `source/LeggedManip_Lab/LeggedManip_Lab/tasks/.../leggedmanip_lab_env_cfg.py` | EventCfg 域随机化 |
| `.../config/go2_piper/wbc_env_cfg.py` | WBC 训练配置（奖励） |
| `.../config/go2_piper/agents/rsl_rl_ppo_cfg.py` | PPO 超参（lr、iterations） |
| `scripts/rsl_rl/train.py` | 训练入口（--finetune_policy） |
| `scripts/rsl_rl/play.py` | 推理 + 导出 policy.pt |

## 6. 部署侧待测试的目标参数（DART world）
- `gz_wbc.world`：DART physics，地面摩擦 mu=2.0
- `gz_ros2_control`：腿 kp=30 / kd=0.6（contract 决定），effort_limits hip/thigh 23.7 Nm
- spawn 高度 0.26~0.29

## 7. 2026-08-20 Gazebo Sim/DART 验证结果
- 新策略 SHA256 `9387bf47f58ea75583f16a91f8d48f44c6200cf2e6fe4339cedd9940edc0c579` 已部署到 Go2Arm_sim2sim。
- Gazebo Sim 集成链路正常：controllers active，WBC ready=true，policy 210/18，sensors_fresh=true。
- 原始 PD kp/kd=30/0.6 站立失败：真实 model pose 在 25s 约 `z=0.221, roll=-0.950, pitch=0.762`。
- PD 兜底 kp/kd=50/1.0 也失败：25s 约 `z=0.305, roll=-3.112, pitch=0.825`，85s 漂移到 `x=2.084, y=-4.250, z=0.186`；effort saturation≈0.1667。
- 结论：finetune 提高了 Isaac/PhysX 训练指标，但没有解决 DART 真实部署稳定性；问题仍是 DART 动力学/观测/控制闭环不匹配。
