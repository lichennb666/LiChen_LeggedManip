# 任务计划：训练能适配 Gazebo Sim（DART）的 WBC 策略

## Goal
当前 WBC 策略（`policy.pt`）在 MuJoCo（73%/78%/54%）和 Gazebo Classic ODE 下能工作，但在 **Gazebo Sim DART 物理下无法站立**（见 Go2Arm_sim2sim/.planning/2026-08-18-gazebo-sim-wbc-migration 的 findings）。本计划通过**增强训练域随机化覆盖 DART 物理特性 + finetune 微调**，训练出能在 Gazebo Sim DART 下站立并响应用户速度指令的 WBC 策略。

## Next Step
Phase 5 验证失败：新策略已加载且 WBC ready，但 Gazebo Sim/DART 仍无法站立；PD 兜底 kp/kd=50/1.0 也失败。下一步做失败复盘，决定是否继续 DART 专项训练（倒地恢复/真实姿态观测/更强接触随机化），或停止 DART 路线。

## Current Phase
Phase 5

## Phases

### Phase 1：DART 与训练环境物理差异分析
- [x] 对比 DART 与 PhysX 的摩擦模型（摩擦锥 vs 库仑）、接触求解（隐式约束 vs LCP）、关节阻尼行为
- [x] 明确现有 EventCfg 域随机化范围与实际 DART 参数的差距
- [x] 确定训练需要覆盖的关键参数及范围（摩擦、执行器增益、关节摩擦/armature、基座姿态扰动）
- [x] 将分析结论写入 findings.md
- **Status:** complete

### Phase 2：增强训练域随机化
- [x] `leggedmanip_lab_env_cfg.py` EventCfg：摩擦范围已扩到 `(0.3, 2.5)`（覆盖 DART world 的 mu=2.0）
- [x] 执行器增益已从 scale 改为 add（刚度 ±2.0、阻尼 ±0.5），让策略对增益敏感度鲁棒
- [x] 已新增关节库仑摩擦/armature 随机化（DART 关节动力学差异大）
- [x] 已加大基座初始姿态/高度扰动（让策略学会落地站稳，对应 spawn 冲击）
- [x] 保留 WBC 现有奖励（stand_still_bonus、低速精跟踪等）
- **Status:** complete

### Phase 3：finetune 微调训练
- [x] `rsl_rl_ppo_cfg.py`：learning_rate=3e-4、max_iterations=4000
- [x] 用现有 `train.py --finetune_policy` 机制（已验证 8/8 权重拷贝成功），从当前 policy.pt 初始化 actor
- [x] smoke run 通过：`python scripts/rsl_rl/train.py --task GO2-PIPER-WBC --num_envs 256 --headless --finetune_policy mujoco/deploy/policy/go2_piper/wbc/policy.pt --max_iterations 100`
- [x] 正式训练完成：`python scripts/rsl_rl/train.py --task GO2-PIPER-WBC --num_envs 1024 --headless --finetune_policy mujoco/deploy/policy/go2_piper/wbc/policy.pt --max_iterations 4000 --run_name dart_domain_4000`
- [x] 观察训练曲线：最终 reward≈156.7、episode_length≈990、bad_orientation≈0.0049、base_contact≈0.0029
- **Status:** complete

### Phase 4：导出与部署
- [x] `play.py` 已导出新策略到 `logs/rsl_rl/go2_piper_wbc/2026-08-20_06-13-26_dart_domain_4000/exported/policy.pt` 和 `policy.onnx`
- [x] 已备份旧策略并覆盖 `mujoco/deploy/policy/go2_piper/wbc/policy.pt`
- [x] 已备份旧策略并覆盖 `Go2Arm_sim2sim/ros2_ws/policy/go2_piper/wbc/policy.pt`
- **Status:** complete

### Phase 5：Gazebo Sim 验证与迭代
- [x] 用 `go2-piper-humble-gz:latest` 镜像 + `gz_wbc.launch.py` 部署新策略
- [x] 验证：DART 下能站立（base z≈0.28 保持 30s 不倒）——失败，25s 起真实 model pose 已明显倾倒
- [ ] 验证：/cmd_vel 前进/横移/转向响应——未执行，因为站立门禁失败
- [ ] 三引擎性能对比（MuJoCo / ODE / DART），写入 findings.md——需先做 MuJoCo/ODE 新策略回归，DART 标记为无法站立
- [x] 若仍失败：回到 Phase 2 调域随机化，或方案 B（部署侧调 PD/接触参数）——已试 PD kp/kd=50/1.0，失败且 effort saturation 升高
- **Status:** in_progress

## 关键决策
- 采用「域随机化增强 + finetune」而非从零训练（保留平地 WBC 已有能力，只增强物理鲁棒性）
- 训练仍用 Isaac Lab（PhysX），不直接换 DART 训练（Isaac Lab 不支持 DART 引擎）
- 域随机化范围覆盖 DART 的关键物理特性，让策略对物理差异鲁棒

## 关联
- 训练侧代码：`source/LeggedManip_Lab/LeggedManip_Lab/`
- 部署侧方案与完整坑记录：`Go2Arm_sim2sim/.planning/2026-08-18-gazebo-sim-wbc-migration/`
