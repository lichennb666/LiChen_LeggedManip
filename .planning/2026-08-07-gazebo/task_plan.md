# 任务计划：Gazebo 启动即方块 + 机械臂中立位 + 键盘遥操作测试

## Goal
1. Gazebo 场景一启动就带绿色方块（world 内置，不再用延迟 spawn）。
2. 机械臂启动位姿改为中立位，避免“零关节→策略自然姿态”的启动扫掠把方块撞下台。
3. 提供一键启动脚本 + launch，在 Gazebo 场景中对 Go2+Piper 用键盘控制运控（/cmd_vel 速度控制）测试。
4.（延伸）修复晃动并验证完整抓取运输 demo。

## Next Step
默认 demo 已跑通并回归通过；纯物理模式当前保留正向调试基线：`PHYSICAL_GRASP=true` 时 mission 从 10s 启动、任务开始后 reset cube 到 `[0.48, 0.16, 0.531]` 且清零 cube twist、物理目标使用 Gazebo 实体中心、physical PREGRASP/DESCEND 使用 station keeping、lift 阶段锁定 TCP x/y。`physical_grasp_lateral_offset=0.012`、`pregrasp_height=0.025/0.030`、仅 PREGRASP 后撤 `physical_pregrasp_forward_offset=-0.02`、finger pad 横向收窄 `0.010 0.05 0.035`、finger pad 增高 `0.015 0.05 0.045`、更深 `physical_gripper_closed=[0.015,-0.015]`、`grasp_height_offset=0.010` 均已验证为负向或未达标并回退/禁用。新增 close 后夹爪几何诊断显示：收紧 DESCEND z 容差 `0.02→0.01` 后，方块相对两指中心线偏差从约 `3.1cm` 改善到约 `1.0cm`；reset x=0.48 可让物理 run 更稳定进入 CLOSE。最新诊断显示：CLOSE 几何可达到 `off_finger_line≈0.002m`，夹爪 effort 未饱和，但 lift 阶段 pad z 仍不随 IK 目标上升，最终 z 误差约 `3.3~3.7cm`。当前瓶颈不是夹爪镜像、WBC 微调、pad 尺寸或单纯闭合目标，而是 lift 阶段接触/约束/末端实体响应问题。下一步应查 lift 时 Link6/Link7/Link8 接触和 Gazebo 约束，而不是继续试几何参数。

## Current Phase
Phase 12（纯物理抓取专项推进）

## Phases

### Phase 1：调研现有实现
- [x] 读 wbc_node.py、convert_legacy_urdf.py，确认臂初始关节位置与启动扫掠来源
- [x] 读 simulation.launch.py / physics_mission.launch.py / world 文件，确认方块 spawn 时序
- [x] 确认容器内 teleop 可用性（自写 termios 节点）
- [x] 将发现写入 findings.md
- **Status:** complete

### Phase 2：world 内置方块
- [x] 把 target_cube 写入 pick_transport.world（启动即在台面上）
- [x] 移除 simulation.launch.py 中的延迟 cube_spawn
- **Status:** complete

### Phase 3：机械臂启动中立位
- [x] spawn 用 -J 指定 18 关节初始位（臂=自然位），wbc_node 新增 startup_target 保持该位姿
- [x] FK 验证自然位不进入方块盒；实际 Gazebo 验证待用户运行
- **Status:** complete

### Phase 4：键盘遥操作测试 launch + 一键脚本
- [x] 新增 teleop_test.launch.py（Gazebo + 控制器 + wbc 策略）
- [x] 新增 teleop_keyboard 节点 + run_teleop_test.sh 一键启动脚本
- **Status:** complete

### Phase 5：验证与交付
- [x] 检查 launch/脚本/XML/Shell 语法与包配置
- [x] 更新 README 使用说明
- [x] 容器内无头端到端验证（方块 105s 不掉台、臂中立位、/cmd_vel 前进）
- **Status:** complete

### Phase 6：晃动与抓取修复（延伸）
- [x] 量化晃动（imu pitch std 0.59 rad/s，臂关节速度 std ~1.2 rad/s）
- [x] PD 控制器 damping_scale（臂 ×2）→ 晃动显著改善（pitch std 0.20，×3 可到 0.04）
- [x] grasp_forward_offset 0.03 + 方块 y=0.13 → 夹取验证通过（cube lift=0.031m）
- [x] 足端锁定尝试与回退（臂运动下不稳）
- [x] 结论：运输失败源于 Gazebo sim2sim 运控漂移，导航路线方案已确认（域侧微调为主）
- **Status:** complete

### Phase 7：第一阶段——cmd_vel 指令适配层
- [x] wbc_node 新增 cmd_vel 限幅/加减速斜坡/平滑停车/死区
- [x] wbc.yaml 增加对应参数
- [x] 重建并在 Gazebo 验证：x=0.4 阶跃 0.4s 斜坡到目标，停止平滑归零
- [x] 重跑任务：抬升阶段仍因 IK 退化失败（运控问题，归第二阶段）
- **Status:** complete

### Phase 8：第二阶段——训练改动清单文档
- [x] 产出 docs/NAV_READY_TRAINING_PLAN_CN.md：域随机化具体范围、站位保持/低速跟踪奖励、7 项验收门禁、微调执行建议
- [x] 更新 README/计划记录
- **Status:** complete

### Phase 9：训练配置落地 + 微调训练（本会话执行）
- [x] mdp/rewards.py 新增 stand_still_bonus、track_lin_vel_low_speed_exp
- [x] wbc_env_cfg.py 挂载两个新奖励（2.0 / 1.5）；leggedmanip_lab_env_cfg.py 扩摩擦/执行器增益/惯量随机化
- [x] rsl_rl_ppo_cfg.py：WBC 学习率 5e-4 → 3e-4（微调）
- [x] train.py 新增 --finetune_policy（从导出 policy.pt 初始化 actor，验证 8/8 权重拷贝成功）
- [x] 确认并复用已配置环境：原 isaac-sim 容器（rsl-rl 5.4.1 / isaaclab 0.54.4），修复 editable 路径指向 /workspace 挂载、补装 git
- [x] 启动微调训练：GO2-PIPER-WBC, 1024 envs, 2000 iters, finetune_from_export（迭代 1.25s，总 44min，奖励 0.02→96→149）
- [x] 导出并部署新策略（mujoco/deploy/policy/go2_piper/wbc/policy.pt，旧策略备份 policy_pretrain_backup.pt）
- **Status:** complete

### Phase 10：训练后验证
- [x] 导出微调策略并部署（备份保留）
- [x] 站立验证：3 分钟位置漂移 <2cm（旧策略漂移明显）——显著改善
- [x] 导航验证：3 航点路线完成，最终误差 1.4cm（基线 8cm）——改善
- [x] 抓取验证：2 次成功（lift=0.041/0.042m；旧策略最好 0.031m 且不稳定）——改善
- [x] 运输验证：方块随行 40s（旧策略一走路即甩），但任务携带导航卡在 0.22m 处——新策略存在低速死区（纯 x<0.2 m/s 不走），任务控制律限制 0.136 m/s 不匹配；已改任务侧（最小速度+横向分量）未及验证
- [x] 偶发问题：方块启动阶段被撞下桌（旧策略亦存在）
- **Status:** complete

### Phase 11：默认 demo 交付 + 物理抓取边界记录
- [x] 修复 SDF initial_position 写入和 ros2_control initial_value 不一致
- [x] 修复 Link8 指垫镜像方向，Link7/Link8 pad 同高对夹
- [x] 加厚夹持方向 collision，保留纯物理模式后续调试入口
- [x] `PHYSICAL_GRASP=false` 默认加载 demo SDF + virtual grasp，完整任务复跑通过
- [x] README 和规划记录说明默认 demo 与纯物理接触模式的边界
- **Status:** complete

### Phase 12：纯物理抓取专项推进
- [x] 收紧 PREGRASP 门限，证明 0.15m 高位 pregrasp 不可达
- [x] 降低 `pregrasp_height` 到 0.04，使物理预抓取进入可达区域
- [x] 缩小 finger pad 前后 footprint 到 `0.025 0.05 0.035`
- [x] 纯物理 lift 验证通过一次（cube lift=0.029m）
- [x] 夹爪力增强实验失败并回退
- [x] 携物速度参数化并降速，默认 demo 回归通过
- [x] 诊断并回退目标刷新/提高抓取高度等负向接近修正
- [x] 诊断并回退平面门限 0.015；确认当前 WBC+IK 不适合继续收紧视觉伺服门限
- [x] 将 finger pad 接近方向 footprint 缩到 `0.015 0.05 0.035`，出现一次物理 lift 通过但运输滑落
- [x] 携物阶段横向速度置零，减少运输扰动；待稳定进入运输后继续验证
- [x] 默认 demo 再次回归通过（运输距离 0.201m）
- [x] 物理模式 mission 启动延迟从 17s 缩到 10s，减少任务前 cube 漂移；默认 demo 保持 17s
- [x] 诊断并回退 pad 第二尺寸 0.035/0.04、PREGRASP 平面 0.04、grasp_height 0.015、桌面高摩擦等负向实验
- [x] 默认 demo 再次回归通过（运输距离 0.209m）
- [x] 物理模式新增任务开始 reset cube，减少启动漂移随机性
- [x] 确认 reset `[0.50, 0.16, 0.531]` 可作为当前复现基准，但仍在接近/闭合阶段推块
- [x] 诊断并回退 `grasp_forward_offset=-0.015/-0.005`，负 offset 会夹空或产生 y 向推块
- [x] 诊断并回退 `pregrasp_height=0.06`，当前 WBC+IK 贴近方块时高位预抓取不可达
- [x] 默认 demo 再次回归通过（`DONE progress=100%`）
- [x] 诊断并回退 `PRECLOSE` 预闭合时序，实测会增加 y 向推块
- [x] 修正 physical lift 的 `vertical_only`：锁定进入 lift 时的 TCP x/y，减少 lift 横向拖拽
- [x] 诊断并回退 `cube_reset_pose=[0.46,0.16,0.531]`，实测 close/lift 更差
- [x] 默认 demo 再次回归通过（运输距离 0.201m）
- [x] 纯物理模式目标改用 Gazebo 实体中心校正，减少感知偏差；默认 demo 回归通过
- [x] 诊断并回退只收紧 grasp 平面门限 0.015，实测未改善且可能加重推块
- [x] 物理 `pregrasp_height` 降到 `0.035`，避开当前 WBC/IK 的 0.04m 可达边界；默认 demo 回归通过
- [x] 新增纯物理专用 `physical_gripper_closed=[0.022,-0.022]`，默认 demo 仍用虚拟抓取闭合目标
- [x] 新增 PREGRASP 后窄条件物理目标刷新，补偿开口指垫推块；实测一次通过 `physical grasp verified: cube lift=0.025m`
- [x] 携物速度降到 `0.10/0.08`，默认 demo 最终回归通过（运输距离 0.203m）
- [x] 物理 `physical_lift_height=0.035` 复跑：仍失败在 `LIFT_STOW`，pad z 误差约 0.035m，未进入运输
- [x] 默认 demo 最新回归通过（`DONE progress=100%`，运输距离 0.203m）
- [x] 增加 lift 诊断日志：确认命令/IK/关节都有动作，pad z 不上升且夹爪状态在 lift 中变化
- [x] 诊断并回退 TCP 开环 lift，实测更差（cube lift=-0.002m）
- [x] 收紧物理 PREGRASP z 容差到 0.025，避免低位假通过
- [x] 诊断并回退 `locomotion_enabled=false` 站立锁定，实测 base 高度塌陷
- [x] physical PREGRASP/DESCEND 使用 station keeping，接近阶段明显改善
- [x] 诊断并回退 `gripper_close_wait=1.2`，实测 close 推块且夹爪仍未有效收敛
- [x] 默认 demo 回归通过（`DONE progress=100%`，运输距离 0.202m）
- [x] 诊断并禁用仅 PREGRASP 后撤 `physical_pregrasp_forward_offset=-0.02`，实测会让 PREGRASP z 误差卡在约 0.038m
- [x] 默认 demo 回归通过（`DONE progress=100%`，运输距离 0.203m）
- [x] 诊断并回退 finger pad 横向收窄 `0.010 0.05 0.035`，接近改善但 lift 仍失败（z 误差约 0.030m）
- [x] 诊断并回退 finger pad 增高 `0.015 0.05 0.045`，close/lift 更差（z 误差约 0.045m）
- [x] 默认 demo 回归通过（`DONE progress=100%`，运输距离 0.203m）
- [x] 新增 CLOSE 后夹爪几何诊断，确认 baseline 失败时方块相对两指中心线偏差约 3.1cm
- [x] 收紧 physical DESCEND z 容差 `0.02 -> 0.01`，close 几何改善到中心线偏差约 1.0cm，但 lift 仍失败（z 误差约 0.048m）
- [x] 默认 demo 回归通过（`DONE progress=100%`，运输距离 0.203m）
- [x] 新增 gripper velocity/effort/close target error 诊断，确认 CLOSE 后夹爪 effort 未饱和
- [x] 物理 reset x 改为 `0.48`，抵消 reset 后漂移并稳定进入 CLOSE；默认 demo 不受影响
- [x] 诊断并回退更深 `physical_gripper_closed=[0.015,-0.015]`，夹爪后期到位但 lift 仍失败
- [x] 诊断并回退 `grasp_height_offset=0.010`，PREGRASP 可达性变差并超时
- [x] 默认 demo 回归通过（`DONE progress=100%`，运输距离 0.203m）
- [ ] 纯物理运输保持仍未通过（方块携物阶段滑落）
- [ ] 纯物理 lift 仍有随机性，尚未稳定每次进入运输
- [ ] 纯物理运输保持仍未通过（最新失败：offset=0.183m，z=0.549m）
- [ ] 定位纯物理 lift 时 joint/TF/接触状态为何 WBC IK 已收敛但 pad z 不上升
- [ ] 处理 close 后夹爪接触姿态：当前一侧指爪顶块时 gripper 关节保持开口，lift 阶段无法形成稳定携物
- **Status:** in_progress

## Decisions Made
| 决策 | 理由 |
|----------|-----------|
| 机器人以自然位 spawn（腿=DEFAULT_ANGLES，臂=自然位），wbc 启动期保持该位姿 | 消除零位→自然位扫掠；自然位不碰方块 |
| world 内联 target_cube 模型，移除延迟 spawn | 场景启动即有方块，且不依赖模型路径 |
| 自写 termios 键盘节点 + 两段式 sh（后台场景 + 前台遥操作） | launch 内节点无法可靠读终端 stdin；20Hz 续发满足 0.2s 指令超时 |
| SDF + initial_position 实现中立位 spawn（Humble 无 -J） | spawn_entity 不支持 -J；构建期回填 ros2_control |
| PD 臂阻尼 ×2（damping_scale） | 晃动大幅改善同时保留臂跟踪速度 |
| grasp_forward_offset +0.03 | 指垫 TF 与碰撞几何 ~2.5cm 前向偏移，不加偏置会夹空 |
| coarse_alignment_threshold 0.15 | 学习式运控在 Gazebo 不是可靠平移伺服，避免不必要的底盘行走 |
| 默认 `PHYSICAL_GRASP=false` 使用 demo-only virtual grasp，`PHYSICAL_GRASP=true` 保留纯物理接触 | 默认路径先保证检测-抓取编排-运输-放置 demo 稳定；纯物理接触仍受 Gazebo ODE、Piper IK 和 WBC 漂移影响，不作为默认演示 |
| `grasp_forward_offset` 回到 0.0 | `-0.02` 会夹在方块后方，`+0.03` 实测会推方块；中间值作为当前保守默认 |
| 不继续单纯加大夹爪力 | 实测 gripper stiffness/effort 增大和 close wait 延长会把方块挤出；物理运输问题更像携物姿态/支撑几何问题 |
| 不把“再次 WBC 微调”作为当前第一优先级 | 默认 demo 已通过；纯物理失败主要集中在夹爪接触几何、低位接近推块、携物扰动。应先把任务层和接触几何收敛到可重复失败点，再决定是否微调策略 |
| 保留镜像夹爪建模 | Link7/Link8 视觉上相反是镜像夹爪的正常表现；当前验证重点是 pad 接触尺寸和闭合/运输稳定性 |
| 物理模式单独提前 mission 启动 | 17s 等待会让轻质 cube 在任务开始前明显漂移；10s 启动能减少漂移，且不影响默认 demo |
| 物理任务开始时 reset cube | 固定方块初始位姿，减少启动漂移随机性，使后续接近/闭合问题可复现 |
| 不继续负向 `grasp_forward_offset` 和更高 `pregrasp_height` | 实测负 offset 夹空/侧推，高位 pregrasp 不可达；继续试会重复已知负向结果 |
| physical lift 锁定 TCP x/y | 原 vertical-only 目标会跟随横漂，导致 lift 阶段横向拖拽；锁定 x/y 后横向漂移明显减小 |
| 不保留 `PRECLOSE` 或 reset x=0.46 | 两者均为负向实验：`PRECLOSE` 增加 y 向扰动，x=0.46 close/lift 更差 |
| 纯物理目标使用 Gazebo 实体中心 | 纯物理接触调试对厘米级误差敏感；该项只用于 `PHYSICAL_GRASP=true`，默认 demo 仍验证 RGB-D 检测链路 |
| 不继续收紧视觉伺服门限 | grasp 平面 0.015 实测负向；剩余瓶颈是 close 接触，不是视觉伺服精度 |
| PREGRASP 后只在漂移超过 2cm 时刷新物理目标 | 低预抓取会偶发推块；窄条件刷新可避免按旧目标夹空，并已实测通过一次 lift |
| 不继续盲目加深物理夹爪闭合 | `[0.022,-0.022]` 已能闭合到位但仍受接触位置影响；完全闭合/加力历史上会挤出方块 |

## Errors Encountered
| 错误 | 解决方案 |
|-------|------------|
