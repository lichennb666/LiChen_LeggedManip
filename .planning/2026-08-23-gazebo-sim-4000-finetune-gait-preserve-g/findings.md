# 发现与决策：4000-iter WBC 前腿步态与导航抓取联合分析

## Requirements（需求）
- 测试和优化对象是 8 月 18 日路线训练出的 4000 iteration finetune WBC，而不是 `LeggedManip_Lab/ros2_ws` 的旧键盘部署。
- 解决两只前脚回缩、像拖地的不自然步态。
- `Go2Arm_sim2sim` 已实现的自动导航抓取能力必须保持，不能以换模型或简化任务换取步态观感。
- 联合分析两个仓库的 planning，避免重复历史负向实验。

## Research Findings（研究发现）

### 1. 权重身份已确认
- `Go2Arm_sim2sim/ros2_ws/policy/go2_piper/wbc/policy.pt`
- `LeggedManip_Lab/mujoco/deploy/policy/go2_piper/wbc/policy.pt`
- `LeggedManip_Lab/logs/rsl_rl/go2_piper_wbc/2026-08-20_06-13-26_dart_domain_4000/exported/policy.pt`

三者 SHA256 均为 `9387bf47f58ea75583f16a91f8d48f44c6200cf2e6fe4339cedd9940edc0c579`，大小均为 1,111,318 bytes。当前 Go2Arm 导航链确实使用 4000-iter 导出策略。

### 2. 必须区分两条 Gazebo 路线
- `2026-08-18-gazebo-sim-wbc-migration` 是 Gazebo Sim 6/DART 实验。暂停启动修复后纯 PD 能站，但 policy 接管仍倾倒，尚未进入可靠导航抓取。
- `2026-08-14-go2arm-ros2-humble` 是完成 Floor1/Nav2/视觉抓取的 Gazebo Classic 11/ODE 主线，已推进到 Phase 18 且全部完成。
- 用户要保留的导航抓取成功证据来自 Classic/ODE 主线，不应被 DART 失败结论覆盖。

### 3. 当前导航时腿部并非 actor 原样输出
- `floor1_navigation.launch.py` 设置 `enable_demo_trot=true`、`contact_trot_blend=1.0`。
- WBC 节点先推理 actor，再用 `four_beat_crawl_foot_offset()` 和腿部 IK 生成 12 个腿关节目标。
- blend=1.0 时运动阶段最终腿目标完全采用四拍 IK 目标；idle 时也用 nominal stance。
- 因此前腿回缩/拖地首先是 gait overlay、IK、PD 和接触闭环问题，不能未经 A/B 就归因于 4000-iter actor。

### 4. 还有独立物理辅助层
- physics SDF 加载 `go2_piper_base_force_assist`。
- 插件施加有界平面力、垂直支撑、roll/pitch/yaw 力矩；纯转向时建立只保留 yaw 自由度的 world-base revolute joint。
- 这不是位姿/速度覆写，但导航抓取成功来自 actor、四拍 IK、PD、力辅助和任务门禁的组合。

### 5. 当前四拍参数没有拖地裕量
- 频率 1.30 Hz、步长 0.10 m、摆动占比 35%、支撑占比 65%、Nav2 最大线速度 0.20 m/s。
- 理论支撑期机身位移为 `0.20 × 0.65 / 1.30 = 0.10 m`，恰好等于完整足端行程。
- 对速度超调、PD 滞后、脚端滑移、IK 可达边界和前后腿载荷差异没有余量；这会表现为支撑脚被向后拖、下一摆动开始时前腿过度回缩。

### 6. 历史门禁不足以否定视觉拖地
- Phase 13 在 0.20 m/s 下记录 FR/FL 峰值抬升约 105.6/106.2 mm、小腿最低净空约 129.1/127.6 mm。
- 峰值能证明腿曾抬起，却不能证明摆动全程无 toe contact、无支撑滑移、无后伸饱和或相位切换拖地。
- 新诊断需计算逐腿 swing-contact ratio、切向滑移、净空 P5/P50/峰值、目标-实际误差和关节极值停留时间。

### 7. 导航抓取金标准
- Floor1 自动目标约 `(3.55, 2.30, yaw=0)`。
- 历史成功包含 Nav2 到桌、RGB-D 约 0.05–0.15 s 稳定识别、视觉站位、Link7+Link8 双指接触、fixed joint、约 43 mm 抬升和载物再次导航约 1.326–1.38 m。
- 视觉微调安全门约为 `|roll|<=0.22`、`|pitch|<=0.28`、`base z>=0.24 m`；约 0.70 rad 内抓取已验证。
- 默认还包含 demo 级预检测 yaw 对齐辅助；步态 A/B 时必须冻结任务层行为。

### 8. 禁止重复的历史错误
- 不回退 flat policy或旧 WBC；二者 Classic 前进明显更差。
- 不简单放大 yaw 命令；响应非线性且方向有状态依赖。
- 不重复 kp/kd=50/1.0、单点调地面 mu 或 spawn z。
- 不直接再跑 4000+ iter 宽域随机化。
- 不只用 `/odom` 或足端峰值判断步态。
- 不收紧 Nav2 终点 yaw 强迫原地转向。
- 不恢复已否决的主动机身盲扫、staging 强门禁或不稳定 world-foot-lock IK。

### 9. 速度与引擎路线的新结论
- MuJoCo 使用同一 policy contract：50 Hz actor、200 Hz physics/PD、action scale 0.25；地面摩擦为 1.5。
- 历史 MuJoCo `vx=0.5` 跟踪约 73%，即约 0.367 m/s；这是首轮应复测的金标准，不以键盘观感代替。
- Classic 的 actor/WBC 顺序测试曾在 `vx=0.5` 下达到约 0.505 m/s，证明 4000-iter actor 在 ODE 中不是天然只能慢走。
- Floor1 当前 DWB 与 velocity smoother 均把前进限制为 0.20 m/s，倒退为 0.12 m/s；限制来自任务步态/稳定性设计，不是 policy contract 上限。
- 因此首选 Classic：它已有完整任务资产，并存在恢复到 0.35–0.50 m/s 的证据基础。DART 尚未通过 actor 接管站立，不适合先承担导航抓取交付风险。
- 推荐使用分场景控制调度：长走廊空载巡航降低四拍 IK blend、让 actor 主导；接近桌面、视觉精调、抓取和载物阶段切回已验证的低速保守参数。
- 不能直接把全任务速度改到 0.5 m/s。应按 0.20→0.28→0.35→0.45/0.50 m/s 逐档验证，并保持近桌自动降速。

### 10. MuJoCo vx=0.50 A/B（同一 scene、contract、50/200 Hz 控制循环）
- 4000-finetune `mujoco/deploy/policy/go2_piper/wbc/policy.pt`，SHA256=`9387bf47…c579`：平均机体系前进 `0.4216 m/s`，标准差 `0.0258`，横向速度绝对均值 `0.0149`，平均机身 z=`0.2934 m`，最大 `|roll|=0.0676`、最大 `|pitch|=0.0293`。
- 原始备份 `mujoco/deploy/policy/go2_piper/wbc/policy_pretrain_backup.pt`，SHA256=`c5f57c79…800e`：平均前进 `0.4077 m/s`，标准差 `0.0235`，横向速度绝对均值 `0.0305`，平均 z=`0.2886 m`，最大 `|roll|=0.0240`、最大 `|pitch|=0.0369`。
- 4000-finetune 在 0.50 命令下约快 3.4%，横向漂移更小，pitch 更小，但 roll 峰值较大；它仍是保留导航抓取能力的唯一合理部署基线。
- 原始备份虽然姿态更平，但不可作为最终回退：历史任务记录显示它无法完成当前 Go2Arm 导航抓取链；本轮不覆盖 4000-finetune。

## Technical Decisions（技术决策）
| 决策 | 理由 |
|----------|-----------|
| 第一轮只做观测增强和 A/B，不改权重 | 当前最终腿目标被 overlay 覆盖，先定位责任层 |
| 基线只从 `Go2Arm_sim2sim` Floor1 入口运行 | 这是用户真正使用且有任务验收记录的系统 |
| actor、overlay 和实际关节三层同时记录 | 区分策略、轨迹生成和执行/接触问题 |
| 速度/步频设计保留明确裕量 | 当前理论值恰好相等，无法吸收闭环误差 |
| 候选模型和配置全部旁路命名 | 防止覆盖唯一已验证的导航抓取金标准 |
| 训练阶段设置条件门 | 只有部署 A/B 证明 actor 自身不足才修改训练奖励 |
| Classic 为主、DART 为可选对照 | 以完成目标的最短可靠路径为准，不为更换引擎而重做已完成任务 |
| 使用速度/任务阶段调度 blend | 同时利用 actor 的巡航能力与现有抓取阶段的稳定配置 |

## Issues Encountered（遇到的问题）
| 问题 | 解决方案 |
|-------|------------|
| “Gazebo Sim”口语可能泛指 Gazebo 部署，而历史计划严格区分 Classic 与 Sim/DART | 以具体 launch/world/physics engine 为准，报告始终标明 Classic/ODE 或 Sim/DART |
| 旧可视化测试启动了错误仓库 | 后续固定工作目录 `/home/lili/Go2Arm_sim2sim/ros2_ws` 并在启动前打印 policy checksum |

### 11. 本轮 Gazebo Classic 受控结果
- 所有策略测试均使用 4000-finetune `9387bf47…c579`，并通过运行时参数将 `decouple_arm_policy_observation=false`，使 Gazebo 使用与 MuJoCo 一致的 210D/18D 策略契约。
- 纯 actor 直行在暖机后 `cmd=0.50 m/s` 达到约 `0.486 m/s`，`z_mean≈0.293 m`，`|roll|_max≈0.037`、`|pitch|_max≈0.053`；这证明前进速度问题不是 4000 权重本身造成的。
- `contact_trot_blend=0.5` 的部分覆盖在 `0.50 m/s` 下明显横向漂移且姿态恶化；全覆盖在 `0.50 m/s` 下约 `0.186–0.25 m/s`，并出现更大下沉/俯仰，不能作为高速巡航方案。
- 固定将 `contact_trot_blend=0` 用于全程导航时，远距离推进较快但在桌前最终转向/进度门禁失败；因此不能只靠固定降 blend。
- 原有全覆盖 Classic Floor1 配置在不改变任务层的情况下通过完整门禁：Nav2 到桌、RGB-D 对正、双指 Link7+Link8 接触、fixed joint 附着、约 `0.038 m` 物理抬升，并输出 `PICK_COMPLETE:cube detected, grasped and physically lifted`。
- 当前可视化候选沿用该已通过任务门禁的全覆盖配置；虽然 `GAZEBO_PLUGIN_PATH` 临时目录只放了抓取插件，但 `LD_LIBRARY_PATH` 仍会使 force-assist 动态库被 Gazebo 隐式加载，正式 launch 必须增加明确开关或从库搜索路径移除它，不能把“目录隔离”误认为已禁用。

### 12. 结论
- 当前最可靠交付物是“4000-finetune + 完整接触步态 + 已验证抓取状态机”的 Classic 基线；它满足任务正确性，但巡航速度仍低于 actor/MuJoCo 金标准。
- 下一步应把阶段调度实现为 WBC 内部的任务状态/距离调度，而不是仅在外部运行时修改参数：空载远距保留 actor 主导，进入转向/桌前/抓取/载物阶段恢复保守覆盖，并为 force-assist 增加显式禁用或限幅选项。

### 13. 已批准实施规格
- 四拍轨迹默认使用 1.60 Hz、40% 摆动、0.10 m 步长和 0.075 m 抬脚高度；支撑期理论位移为 0.075 m，较完整足端行程保留 25% 裕量。
- 摆动轨迹改为前 20% 垂直离地、中间 60% 平移、后 20% 平滑落脚，并使用端点零速度曲线。
- WBC 增加上一有效 IK 解种子、5 mm 残差/0.05 rad 限位保护、每周期 0.04 rad 目标变化限制，以及逐腿摆动接触/滑移/净空和目标链遥测。
- 模式调度采用 FAST_POLICY/CONTACT_WALK 带迟滞状态机；快速直行才 actor 主导，转弯、横移、低速、靠桌、抓取和负载保持接触步态。
- force-assist 分为关闭、姿态高度保持、完整平面助力三个显式模式，防止直行时平面力把机体推过支撑脚工作空间。
- 助力插件不能继续直接消费原始 /cmd_vel：Nav2 提速到 0.35 m/s 后，CONTACT_WALK 脚端仍按 0.20 m/s 设计，原始命令会再次把机身推过支撑行程。实现改为 WBC 发布限幅后的 /go2_piper/wbc/assist_cmd_vel，插件只消费该话题。
- actor 的隐式步态相位不可由四拍 overlay 相位代替；逐腿摆动拖曳比只在 CONTACT_WALK 且 blend 有效时统计，FAST_POLICY 通过实际足端净空、姿态和速度测试，不伪造摆动相位指标。

### 14. 首轮实现动态结果
- 0.20 m/s CONTACT_WALK 在 14 s 稳定门禁中无姿态/高度失败，最大 roll=0.0473 rad、最大 pitch=0.1084 rad、最低 base z=0.2419 m。
- FR 核心摆动净空 P5=22.1 mm，FR/FL 拖曳估计均为 0；说明“先抬后摆”方向有效。
- 速度只有约 0.029 m/s，不能交付。末端 FL/RL 目标残差为 21.6/33.5 mm，实际跟踪误差也较大，说明安全目标限速/IK 保护与动力层组合过度保守。
- 离线 501 点周期扫描显示四腿目标本身可达，原始 IK 最大残差小于 0.2 mm、最小关节余量大于 0.54 rad；主要不连续只发生在步态启动时。因此应优化启动渐入和动态目标跟踪，而不是缩短步长或放宽关节限位。
- 当前 WBC 已保存 actor 原始动作并支持 JSONL trace，但没有保存 actor 腿目标、IK 腿目标、上一有效 IK 解或逐腿 IK 残差；这些可在同一 50 Hz 更新循环内补齐。
- 当前 gait gate 仅对 FR/FL 的峰值抬脚、全程最小 calf clearance、总位移和姿态设门禁；必须扩展为逐腿后伸/IK/目标跟踪指标。Gazebo 现有模型未暴露逐足 contact topic，因此首轮用足端高度与切向速度联合判定疑似拖曳，并把真实 contact 接口作为插件可用时增强项。
- force-assist 已订阅 locomotion_enabled，适合增加 UInt8 assist_mode 而不改变任务状态机；插件的姿态/高度力和 XY/yaw 力目前在同一 enabled 条件下，需要拆分执行。

### 15. 跟踪修正与 FAST_POLICY 动态证据
- 将 WBC gate 从 5 Hz 提高到 20 Hz，并把命令超时从 0.20 s 调到 0.30 s 后，消除了采样周期恰好撞上超时边界造成的假失效；四腿 IK 残差降到 0.08 mm 以下。
- 将轨迹调整到 1.45 Hz、40% 摆动、15% 垂直抬升、15% 落地，并把目标 slew 提高到 6 rad/s 后，前腿核心摆动净空仍约 18–19 mm、拖曳估计为 0；说明先抬后摆修正确实改善前足离地。
- 0.20 m/s CONTACT_WALK 的净位移仍只有约 0.21–0.25 m/8 s。插件遥测证明它已接收 0.20 m/s 指令、峰值平面力约 66.5 N、瞬时速度约 0.177 m/s；低净速度来自全接触步态下的周期性接触/回摆，而不是插件断连。
- 0.35 m/s 命令能正确触发 CONTACT_WALK→FAST_POLICY，并发布 assist mode 1；任务/转弯条件仍会立即退出高速模式。
- FAST_POLICY 完全关闭平面力时，0.50 m/s 指令的最大实际速度约 0.265 m/s，5 s 净位移约 0.211 m，低于 MuJoCo 和纯 actor 基线。
- mode 1 若同时关闭 yaw 阻尼，0.50 m/s 下 yaw span 达 0.749 rad；恢复 yaw 阻尼后降到 0.084 rad，横向位移约 0.3 mm，证明姿态辅助必须保留。
- mode 1 的 30/60/100 N A/B 都能产生约 0.19–0.39 m/s 的瞬时速度，但 5–8 s 净位移仅约 0.19–0.23 m；提高力上限不能消除足地接触造成的往复抵消。
- 从启动即关闭 overlay 的纯 actor 对照，0.50 m/s 下峰值速度约 0.457 m/s，但 5 s 净位移仍只有 0.230 m；因此此前约 0.486 m/s 的 Classic 结果是瞬时/短窗速度，不能与 MuJoCo 的持续均速 0.4216 m/s 等价。
- 自动 FAST_POLICY 与 0.35 m/s Nav2 组合在 Floor1 两次都于走廊中段触发无有效轨迹/规划恢复失败；它保留为实验功能，但 Floor1 默认关闭。
- 把 Nav2 的 `max_vel_x`、`max_speed_xy` 和 velocity smoother 上限从 0.35 恢复为 0.20 m/s，使规划器与 CONTACT_WALK/assist 执行包络重新一致后，完整任务通过。

### 16. 最新完整任务硬门禁
- 使用冻结 policy `9387bf47…c579`、修正后的三段摆动轨迹、1.45 Hz/40% 摆动/15% 抬升落地、adaptive 默认关闭、Nav2 0.20 m/s。
- Nav2 输出 `Goal succeeded`，随后进入 `TABLE_REACHED`。
- RGB-D 粗对正后 1.33 s 获取目标；视觉站位与机械臂 IK 正常。
- Link7+Link8 双指真实接触后 fixed joint 附着。
- 方块从 z=0.531 m 抬到约 0.567 m，物理抬升 0.036 m；基座解锁后输出 `PICK_COMPLETE:cube detected, grasped and physically lifted`。
- 前足/步态状态在任务回合中报告 swing drag ratio 四腿均为 0；IK 残差为亚毫米级。任务完成耗时明显较长，持续速度仍未达到 MuJoCo，不能把这一项标为完成。

### 17. 最终混合候选与重复性验收
- 纯三段“先抬—平移—落地”轨迹在直行前足净空上通过，但完整任务仅 1/2：一次成功约 366 s，一次在接近桌面时触发 `NAVIGATION_FAILED:6`。因此它不满足任务零回归要求。
- 从历史 `leg_kinematics.cpython-38.pyc` 恢复任务成功版本的 35% swing 转弯公式。最终实现按命令曲率切换：直行使用新三段轨迹，转弯使用旧连续扫掠轨迹；策略权重、任务状态机和抓取链均不改变。
- 混合候选短弧线门禁：5 s 内 `dx=0.205 m`、`dy=-0.0014 m`、`dyaw=0.193 rad`，最大 `|roll|=0.0525`、最大 `|pitch|=0.0947`；摆动核心净空 P5 为 18.2–21.4 mm，四腿拖曳估计均为 0，IK 残差约 0.1 mm。
- 三个相互独立的干净 Floor1 回合全部通过，导航时间约 171–187 s；均出现 `Goal succeeded`、`TABLE_REACHED`、RGB-D 定位、Link7+Link8 双指真实接触 fixed joint、方块物理抬升 34–85 mm、基座解锁与 `PICK_COMPLETE`。
- 第 3 回合明确记录：从 `(-4.03,-2.00)` 出生，约 176 s 到桌，RGB-D 1.04 s 获取目标，方块抬升 34 mm。启动前发现的一次旧容器污染回合已作废，未计入 3/3。
- 结论边界：前腿拖曳问题在直行/短弧线定量门禁中已消除，导航抓取重复性已恢复；但 Gazebo 的持续净速度仍远低于 MuJoCo 0.50 命令下 0.4216 m/s，0.35/0.50 速度目标未完成，不能宣称速度已与 MuJoCo 对齐。

### 18. 高速转身根因与 Phase 7 修正
- 原高速调度把任何 `|yaw|>=0.04` 的命令立即切回 CONTACT_WALK，且切换时把四拍相位硬重置到 0；因此键盘 Q/E 和 Nav2 转弯都会先发生固定的 FR 起步载荷转移，再逐足慢转。
- force-assist 又把所有纯转身（包括 FAST_POLICY mode 1）都识别为 `pure_turn`，动态建立 world-base revolute hinge 并增加 60 N 垂直卸载；这解释了用户观察到的“先晃一下再慢慢转”。
- 把 angular scale 从 0.4 提到 0.8 并不会增大足端转身步幅，因为归一化 yaw 仍为 1；同时 yaw torque 原本在 5 N·m 饱和，命令上限继续提高收益有限。
- Phase 7 修正方向：非保守阶段允许纯转身/弧线进入 FAST_POLICY；FAST mode 不再建立 hinge 或卸载；保守 CONTACT 切换保留连续相位并加快 blend；yaw torque 先从 8 N·m 门禁，再与 12 N·m 对照。
- `TABLE_REACHED/ALIGN/PICK/...` 已由 mission state 自动置为 conservative，因此 Nav2 高速可以保持到桌边，进入视觉/抓取后仍会切 CONTACT_WALK，不需要另造距离估计器。
- 8 N·m 完全自由浮动 FAST 转身会快速侧翻，证明 hinge 的 yaw-free 姿态约束仍必要；引起起步晃动的主要是 60 N 卸载和 CONTACT 四拍切换，而不是 hinge 本身。
- 保留 hinge、FAST 不卸载后，8 N·m 与 12 N·m 均通过短门禁；12 N·m 在相近姿态峰值下把 3 s 净转角从 1.240 提到 1.411 rad（约 +14%），因此采用 12 N·m。

### 19. Phase 7 完整任务终验
- 高速默认配置在干净 Floor1 世界完成一轮导航抓取：导航约 150.3 s，相比旧最终候选的 171–187 s 缩短约 12–20%。
- 到桌后任务进入保守模式；RGB-D 约 0.10 s 获取目标，Link7+Link8 双指接触建立 fixed joint，方块物理抬升 0.039 m，机身解锁并输出 `PICK_COMPLETE`。
- 从 `TABLE_REACHED` 到 `PICK_COMPLETE` 约 26.5 s，证明提高视觉/对正限速没有破坏抓取门禁。
- 任务中 DWB 仍出现多次 `No valid trajectories`、一次进度恢复及全局清图，但最终自恢复并到达；因此可以声明任务速度有提升，不能声明导航全程稳定保持 0.50 m/s。
- 最终复核：policy SHA256 仍为 `9387bf47…c579`，相关测试 25/25 通过，Docker 运行容器为空。

## Resources（资源）
- 训练计划：`LeggedManip_Lab/.planning/2026-08-18-gazebo-sim-dart-training/`
- DART 部署计划：`Go2Arm_sim2sim/.planning/2026-08-18-gazebo-sim-wbc-migration/`
- Floor1 任务计划：`Go2Arm_sim2sim/.planning/2026-08-14-go2arm-ros2-humble/`
- 部署 WBC：`Go2Arm_sim2sim/ros2_ws/src/go2_piper_wbc/go2_piper_wbc/wbc_node.py`
- 四拍轨迹：`Go2Arm_sim2sim/ros2_ws/src/go2_piper_wbc/go2_piper_wbc/leg_kinematics.py`
- Floor1 入口：`Go2Arm_sim2sim/ros2_ws/src/go2_piper_bringup/launch/floor1_navigation.launch.py`
# Phase 8 最终结论（2026-08-24）

- “配置 0.50 m/s 但实际很慢”的直接原因不是策略上限，而是 DWB 在局部代价地图中反复判定轨迹碰撞。基线导航 87.51 s，命令/实速中位数仅 0.049/0.056 m/s，并有 19 次 `No valid trajectories`。
- RPP 更适合这条窄走廊的连续前向路径：首批回合导航约 20.8–24.0 s，命令中位数 0.50 m/s、运动实速中位数约 0.476 m/s。
- 起步仍偶发 collision 的根因是二维激光实际向下 45° 扫地。仅在 URDF sensor 局部 pose 反向补偿会被 `gz sdf -p` 固定关节折叠丢弃，必须在最终 SDF 生成树中归一传感器姿态。
- 修复后 `/scan` 的 0.70–1.20 m 假障碍点为 0；最终导航 18.67 s，无碰撞等待/恢复清图/无有效轨迹报错。
- 最终抓取保留安全门：Link7 与 Link8 两侧真实接触后才建 fixed joint，方块实际抬升 0.066 m，TABLE_REACHED 到 PICK_COMPLETE 12.42 s。
- 未采用的实验：缩小局部 footprint 曾导致到桌时 roll=1.529 rad；全局 inflation 0.50 m 会堵死窄通道；因此最终保留完整 0.70×0.56 m footprint 和全局 inflation 0.38 m。

## Phase 9 初始审计（2026-08-24）

- 当前物理抬升目标仅为 `physical_lift_height=0.040 m`，实际最终回合因抓取几何变化抬升了 0.066 m；将目标提高到 0.080 m 可把方块中心提升到约 0.63–0.64 m，同时仍在当前机械臂已使用的工作空间内。
- `finish_after_pick` 已经执行 `_stop()`、捕获基座相对持物位姿、持续发布持物末端目标、解锁基座并重新启用 locomotion；因此四足控制链具备零速度正常站立的基础，但动作在解锁后立即返回，缺少显式站立稳定时间和最终持物复核。
- 最小改动是保留双指 fixed joint，在抬升验证后解锁基座并持续零速度/相对机身持物目标约 2 s，再检查物块仍被保持并记录最终物理姿态；不进入返程、不收回或松开机械臂。
- 最终可视化回归通过：方块从约 0.528 m 抬到 0.637 m，实际抬升 0.109 m；站立稳定后方块 z=0.647 m、机身 z=0.296 m，双指 fixed joint 仍连接，输出 `post-pick normal standing verified; payload remains held` 和 `PICK_COMPLETE`。

## Phase 10 持物移动实测（2026-08-24）

- 为避开正前方桌子，测试方向选择沿机身反向退入走廊开阔区，并在每段结束持续发布零速度 1.5 s。
- 第一段命令 0.25 m/s、2.5 s，实际位移 0.375 m（约 0.15 m/s）；持物保守控制导致实速低于命令。方块 z 仅从 0.636 到 0.629 m，roll/pitch 峰值 0.061/0.103 rad。
- 第二段在已离桌的开阔区用 0.40 m/s 输入标定，2.0 s 位移 0.462 m，实际约 0.231 m/s，接近用户要求的 0.25 m/s。方块 z 从 0.633 到 0.638 m，roll/pitch 峰值 0.059/0.126 rad，最终 roll/pitch=0.014/-0.069 rad；未掉块并已停车。
