# 操作日志：训练能适配 DART 的 WBC 策略

## 初始状态（2026-08-18）
- 本计划为训练侧新计划，尚未开始执行训练
- 部署侧迁移已完成但策略在 DART 下无法站立（详见 Go2Arm_sim2sim/.planning/2026-08-18-gazebo-sim-wbc-migration/）
- 已具备的复用资产：
  - finetune 机制 `--finetune_policy`（已验证）
  - 训练环境 isaac-sim 容器（rsl-rl 5.4.1 / isaaclab 0.54.4）
  - 历史 finetune 经验（GO2-PIPER-WBC 1024 envs 2000 iters，奖励 0.02→96→149）

## 待执行
- [ ] Phase 1：DART vs PhysX 差异分析
- [ ] Phase 2：域随机化增强（摩擦/执行器增益/关节摩擦/基座扰动）
- [ ] Phase 3：finetune 训练（lr 3e-4, 4000 iters）
- [ ] Phase 4：导出 + 部署（mujoco + ros2_ws policy.pt）
- [ ] Phase 5：Gazebo Sim 验证 + 三引擎对比

## 备注
- 训练前备份当前 `mujoco/deploy/policy/go2_piper/wbc/policy.pt`（参考旧备份 policy_pretrain_backup.pt 的做法）

## 2026-08-20 恢复与 smoke run 准备
- [x] 切换到训练工程 `/home/lili/LeggedManip_Lab`，确认 DART training 计划存在。
- [x] 确认 Phase 1 findings 已记录 DART/PhysX 差异和训练域随机化目标。
- [x] 确认 Phase 2 相关代码已在 dirty diff 中实现：摩擦 0.3~2.5、执行器增益 add、关节 friction/armature、base 姿态扰动。
- [ ] 下一步执行 100 iter smoke run，确认训练入口和 finetune 稳定。
- [x] smoke run 第一次启动失败：容器缺少 `git`，`rsl_rl.utils.logger` 导入 GitPython 时触发 `Bad git executable`。训练尚未开始。
- [ ] 处理：运行 `/workspace/LeggedManip_Lab/docker/setup_container.sh` 修复容器依赖后重试。
- [x] 已运行容器修复脚本并以 root 安装 git；`isaaclab/rsl-rl/gymnasium/git` 检查通过。
- [x] 100 iter smoke run 通过：`GO2-PIPER-WBC`、256 envs、`--finetune_policy mujoco/deploy/policy/go2_piper/wbc/policy.pt`。
- [x] smoke 关键结果：环境创建成功，EventCfg 中 `randomize_joint_parameters`/`randomize_rigid_body_inertia`/`randomize_actuator_gains` 生效；观测 policy shape 210；动作 shape 18；actor 权重拷贝 8/8；生成 `logs/rsl_rl/go2_piper_wbc/2026-08-20_06-05-27/model_99.pt`。
- [x] 修复容器 `/tmp/LeggedManip_Lab` 和 `/tmp/IsaacLab` ownership 为 `isaac-sim:isaac-sim`，避免正式训练前重复权限报错。
- [ ] 下一步启动正式 4000 iter finetune。
- [x] 正式 finetune 已启动：`GO2-PIPER-WBC`、1024 envs、4000 iter、run_name=`dart_domain_4000`。
- [x] 训练日志目录：`logs/rsl_rl/go2_piper_wbc/2026-08-20_06-13-26_dart_domain_4000`。
- [x] 已生成第一个 checkpoint：`model_1000.pt`；约 1056/4000 时 reward≈134、episode_length≈996，训练稳定。
- [x] 已生成 `model_2000.pt`；约 2016/4000 时 reward≈153、episode_length≈991，训练中段稳定。
- [x] 已生成 `model_3000.pt`；约 3028/4000 时 reward≈156、episode_length≈995，进入最后 1000 iter。
- [x] 正式训练完成：`model_3999.pt`，训练时间 4964.66s，最终 reward≈156.71、episode_length≈990.33、bad_orientation≈0.0049、base_contact≈0.0029。
- [x] 导出成功：`exported/policy.pt`、`exported/policy.onnx`。
- [x] 已备份并覆盖部署策略：LeggedManip_Lab MuJoCo WBC policy 和 Go2Arm_sim2sim ROS2 WBC policy；新 policy SHA256=`9387bf47f58ea75583f16a91f8d48f44c6200cf2e6fe4339cedd9940edc0c579`。
- [ ] 下一步：Gazebo Sim/DART 站立与 `/cmd_vel` 验证。
- [x] Gazebo Sim/DART 验证完成：新策略加载成功但站立失败；/cmd_vel 测试因站立门禁失败未执行。
- [x] 方案 B PD 试验完成并失败：kp/kd=50/1.0 不能扶正，已在 Go2Arm_sim2sim 恢复原始 30/0.6。
- [ ] 下一步：失败复盘，决定是否继续 DART 专项训练或结束 DART 路线。
