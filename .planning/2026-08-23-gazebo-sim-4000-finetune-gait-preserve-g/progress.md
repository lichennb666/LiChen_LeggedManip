# 进度日志

## Session: 2026-08-23

### Current Status
- **Phase:** 9 - 更高抬升与持物站立结束姿态（完成）
- **Started:** 2026-08-23

### Actions Taken（已执行操作）
- 已实现四拍 40% 摆动与三段先抬后摆轨迹，并新增独立 AdaptiveGaitMode 状态机。
- 已接入 WBC：4000 模型 SHA256 启动校验、FAST_POLICY/CONTACT_WALK 调度、上一有效 IK 目标、5 mm 残差与 0.05 rad 限位保护、2 rad/s 目标 slew、任务阶段保守锁存及逐腿遥测。
- 已将 force-assist 拆成 0=关闭、1=仅姿态高度、2=完整助力，并改为消费 WBC 限幅后的 assist_cmd_vel；快速策略直行不再被平面力推着走。
- 已把 Floor1 巡航上限提高到 0.35 m/s；最终任务验收默认 finish_after_pick=false，以覆盖载物退桌和返程。
- 修复 WBC gate 5 Hz 与 0.20 s command timeout 的边界混叠：gate 改为 20 Hz，timeout 改为 0.30 s，动态 IK 残差降到 0.08 mm 以下。
- 根据 Gazebo 动态带宽把候选修正为 1.45 Hz、40% 摆动、15% 抬升/15% 落地、6 rad/s 目标 slew；前腿核心摆动净空约 18–19 mm、拖曳估计为 0。
- 验证 force-assist 连接正常；完全关闭 FAST_POLICY 平面助力速度不足，而关闭 yaw 阻尼会产生明显摆头。
- 完成 FAST_POLICY 30/60/100 N A/B：峰值速度可提高，但持续净位移没有随助力改善，否决把加力作为速度交付方案。
- 完成从启动即纯 actor 的 0.50 m/s 对照：峰值约 0.457 m/s，5 s 净位移约 0.230 m；修正历史“0.486 m/s”为瞬时速度结论。
- 两次 0.35 m/s Floor1 自动高速回归在走廊中段失败；自适应调度功能保留，但默认配置关闭。
- Nav2 与 CONTACT_WALK 重新统一到 0.20 m/s 后，完整回归通过：Goal succeeded、TABLE_REACHED、RGB-D、双指 fixed joint、0.036 m 物理抬升、PICK_COMPLETE。
- 相关单元测试最终 23/23 通过；部署 policy SHA256 仍为 `9387bf47f58ea75583f16a91f8d48f44c6200cf2e6fe4339cedd9940edc0c579`；测试容器和 Gazebo 进程已清空。
- 用户批准实施前腿拖地/回缩修正方案；本轮进入部署侧实现和回归阶段。
- 从旧 Python 3.8 字节码恢复了任务历史成功时的 35% 摆动转弯足端公式，并实现混合调度：直行使用三段先抬后摆轨迹，转弯使用旧轨迹。
- 混合候选弧线门禁通过：5 s 净位移约 0.205 m、净转角约 0.193 rad，最大 roll/pitch 约 0.053/0.095 rad，四腿拖曳估计为 0，摆动核心净空 P5 约 18.2–21.4 mm。
- 最终混合候选完成 3/3 个干净 Floor1 导航抓取回合；导航约 171–187 s，均到桌并完成 RGB-D、Link7+Link8 双接触 fixed joint、34–85 mm 方块抬升、基座解锁和 `PICK_COMPLETE`。
- 一次启动被遗留 Gazebo 容器污染（旧实体已在目标附近），明确作废且不计成功率；清空容器后重新执行第 3 次干净回合并通过。
- 转弯遥测摆动相位已对齐旧轨迹固定 35% 占比；相关单元测试 24/24 通过。
- Phase 7 将非保守纯转身/弧线纳入 FAST_POLICY，并让 TABLE_REACHED/ALIGN/PICK 等任务状态继续作为 CONTACT_WALK 安全切换门；取消 FAST→CONTACT 的四拍相位硬重置，contact hold 从 1.0s 缩至 0.25s、blend rise 从 2/s 提至 4/s。
- 首个 8 N·m“完全取消 hinge”对照转得快但失稳（roll 峰值>1 rad），否决该结构；修正为 FAST 仍保留 yaw-free hinge，但不做 CONTACT 的垂直卸载。
- 修正后 8 N·m：3 s 净转角 1.240 rad，roll/pitch 峰值 0.088/0.083 rad，平面漂移约 1.1 mm，门禁通过。
- 同条件 12 N·m：3 s 净转角 1.411 rad，roll/pitch 峰值 0.085/0.082 rad，平面漂移约 0.5 mm，门禁通过；比 8 N·m 快约 14%，选择 12 N·m。
- 任务微调速度已提高：粗对正 0.16/0.25 m/s(rad/s)，视觉搜索 0.12/0.18，精对正 0.18/0.18、纯转 0.25；导航默认 0.50 m/s、0.80 rad/s。
- 恢复活动计划并确认工作树包含用户既有改动；本轮只修改 Go2Arm 步态/WBC/插件/测试及本计划文件，不触碰 LeggedManip 训练与模型文件。
- 锁定实施参数：1.60 Hz、40% 摆动、0.10 m 步长、0.075 m 抬脚，三段先抬后摆轨迹，4000-finetune 模型哈希保持不变。
- 创建跨仓库专项计划并切换为活动计划。
- 读取 `LeggedManip_Lab` 8 月 18 日训练计划及 `Go2Arm_sim2sim` 8 月 14/18 日任务与迁移计划。
- 校验三个关键 policy 文件哈希，确认 Go2Arm 当前部署与 4000-iter 导出均为 `9387bf47…c579`。
- 审计 Floor1 launch、WBC 配置、actor 观测/动作、四拍足端轨迹、IK blend、base force assist 和导航抓取历史门禁。
- 纠正此前测试对象错误：`LeggedManip_Lab/ros2_ws` 的手动 Gazebo Classic 入口不是本任务验收入口。
- 明确 Floor1 运动阶段 `contact_trot_blend=1.0`，部署侧四拍 IK 完全覆盖 actor 腿目标；将部署侧定量诊断置于训练之前。
- 汇总 flat/旧 WBC 回退、PD 增益、摩擦/spawn、简单 yaw gain、盲扫和严格原地转向等历史负向方案。
- 对照 MuJoCo 部署契约与 Floor1 Nav2 参数：确认同为 50 Hz policy/200 Hz PD，而 Floor1 前进速度被 DWB 和 smoother 限制为 0.20 m/s。
- 根据用户允许任选 Gazebo 引擎的要求，选择 Classic/ODE 为首轮主交付路线；DART 暂作为后续可选对照。
- 将优化结构修正为分场景控制：空载长距离巡航 actor 主导，近桌/抓取/载物阶段保持保守模式。
- 创建临时无头 MuJoCo A/B 测试脚本，固定相同 scene、policy contract、50 Hz actor/200 Hz PD 和 `vx=0.50 m/s`。
- MuJoCo 4000-finetune 结果：平均 `0.4216 m/s`，最大 `|roll|=0.0676`，最大 `|pitch|=0.0293`。
- MuJoCo 原始备份结果：平均 `0.4077 m/s`，最大 `|roll|=0.0240`，最大 `|pitch|=0.0369`。
- 选择 4000-finetune 作为后续 Gazebo 基线：速度更高、横向漂移更小，且保留已验证的导航抓取任务能力；原始备份仅保留为 A/B 参考。
- 运行时关闭 `decouple_arm_policy_observation` 后，Gazebo Classic 的 4000 策略契约与 MuJoCo 对齐；未对权重做覆盖或替换。
- Gazebo Classic 受控测速：纯 actor 暖机后 `cmd=0.50` 约 `0.486 m/s`，姿态 `|roll|_max≈0.037`、`|pitch|_max≈0.053`；部分/全接触覆盖速度和姿态明显更差。
- 全接触 Floor1 任务回归通过：`Goal succeeded` → `TABLE_REACHED` → 双指接触 fixed joint → `assisted grasp lift verified: cube lift=0.038m` → `PICK_COMPLETE`。
- 固定 `contact_trot_blend=0` 的全程 actor 主导导航在最后转向失败；外部距离切换候选也未通过最终导航门禁，暂不交付为默认配置。
- 已启动最终 Gazebo Classic 可视化候选（4000-finetune、全接触、任务门禁通过配置）；检查发现即使临时缩小 `GAZEBO_PLUGIN_PATH`，`LD_LIBRARY_PATH` 仍会隐式加载 force-assist，后续必须在正式 launch 增加显式开关。
- Phase 8 DWB 基线一次导航 87.51 s：命令前进速度中位数 0.049 m/s、实速中位数 0.056 m/s，出现 19 次 `No valid trajectories`、2 次进度失败与清图恢复；另一次约 136 s，说明停顿具有随机性。
- 局部控制器改为 Regulated Pure Pursuit 后，首批干净回合导航约 20.8–24.0 s；命令速度中位数 0.50 m/s，运动实速中位数 0.476 m/s，DWB 的 `No valid trajectories` 完全消失。
- 进一步定位到起步残余碰撞的物理根因：`laser_livox` 固定关节带 45° 俯角，URDF 转 SDF 的固定关节折叠使二维导航激光射线扫到地面，形成约 0.47 m 的对称假墙。
- 在最终 SDF 生成步骤把 `navigation_laser` 归一为水平姿态；修复后 `/scan` 中会进入障碍层的 0.70–1.20 m 假点数量为 0，剩余 0.15–0.20 m 自身回波低于障碍层最小量程。
- 最终完整 Floor1 回归：AUTO_GOAL 到 Goal succeeded 为 18.67 s；TABLE_REACHED 到 PICK_COMPLETE 为 12.42 s；无碰撞等待、无清图、无 `No valid trajectories`；Link7+Link8 双接触 fixed joint，方块实际抬升 0.066 m。
- 修正 Floor1 集成入口的终止语义为 `finish_after_pick=true`。该入口到桌后才启动任务，旧默认错误地把桌边当返程起点，再套用独立搬运的 1.30 m 门禁，导致已抬升 0.063 m 的抓取被误报失败。
- 最终 WBC 相关测试 25/25、mission 测试 16/16 通过，YAML 可解析；部署权重 SHA256 仍为 `9387bf47f58ea75583f16a91f8d48f44c6200cf2e6fe4339cedd9940edc0c579`；运行中的 Docker/Gazebo 容器已清空。
- Phase 9 将 `physical_lift_height` 从 0.040 m 提到 0.080 m，并在 `finish_after_pick` 解锁机身后增加 2 s 零速度正常站立稳定段；期间持续发布基座相对持物目标且不松开双指 fixed joint。
- Phase 9 可视化完整回归通过：导航正常到桌，Link7+Link8 fixed joint，方块实际抬升 0.109 m；最终 picked_standing 时方块 z=0.647 m、机身 z=0.296 m，持物复核通过并输出 `PICK_COMPLETE`。Gazebo GUI 保持运行供用户观察。
- 修改后 mission 单元测试 16/16 通过，mission YAML 解析正常。
- Phase 10 避开桌面向后完成两段持物移动：首段命令 0.25 m/s 的实际速度约 0.15 m/s；开阔区标定段实际约 0.231 m/s，方块高度稳定在约 0.63 m，姿态峰值保持在 roll 0.059/pitch 0.126 rad，结束后自动停车并继续持物。

### Test Results（测试结果）
| 测试 | 预期 | 实际 | 状态 |
|------|----------|--------|--------|
| Phase 7 高速完整导航抓取 | 高速策略接近桌边，再用 CONTACT_WALK 对正并抓取 | 导航约 150.3s，`Goal succeeded`/`TABLE_REACHED`；RGB-D 0.10s；双指 fixed joint；cube lift=0.039m；`PICK_COMPLETE` | 通过 |
| Phase 7 纯转身 8/12 N·m | 选择姿态稳定的更快档 | 8 N·m=1.240rad/3s；12 N·m=1.411rad/3s，roll/pitch 峰值=0.085/0.082rad | 通过，选 12 N·m |
| Phase 7 最终源码回归 | 当前工作树保持模式、轨迹与 policy contract | `test_gait_control`/`test_leg_kinematics`/`test_policy_core` 合计 25 passed | 通过 |
| Go2Arm policy 与 4000-iter 导出哈希 | 完全一致 | 三处均为 `9387bf47…c579` | 通过 |
| 新轨迹、旧转弯轨迹与模式状态机单元测试 | 抬脚先于水平转移，旧公式精确复现，模式迟滞正确 | 24 passed | 通过 |
| 混合候选短弧线 | 转弯推进、姿态和前足净空不退化 | dx=0.205m、dyaw=0.193rad、roll/pitch 峰值=0.053/0.095rad、拖曳比=0 | 通过 |
| 混合候选完整导航抓取重复性 | 3 个干净世界均完成导航、视觉、双指附着与物理抬升 | 3/3 成功；导航约 171–187s；抬升约 34–85mm | 通过 |
| 修正后 Gazebo 0.20 m/s CONTACT_WALK 初测 | 不倒、前腿不拖且接近命令速度 | 稳定门禁通过，FR 核心摆动 P5=22.1 mm、拖曳比=0；但 8 s 仅 0.230 m（约 0.029 m/s），FL/RL IK 跟踪残差偏高 | 未通过速度门禁，继续修正 |
| gate/timeout 修正后的 CONTACT_WALK | 排除命令采样与 IK 保护伪故障 | 四腿 IK 残差均不超过 0.08 mm；前腿核心净空约 18–19 mm、拖曳估计 0；净速度仍偏低 | 步态指标通过，速度未通过 |
| FAST_POLICY 0.50 m/s（无 XY 助力、保留 yaw 阻尼） | 接近 MuJoCo 0.4216 m/s 且直行稳定 | 最大速度约 0.265 m/s，5 s 净位移约 0.211 m；yaw span=0.084 rad、横漂约 0.3 mm | 姿态通过，速度未通过 |
| FAST_POLICY 30/60/100 N 与纯 actor 对照 | 找到可持续高速候选 | 峰值约 0.19/0.34/0.39/0.457 m/s，但 5–8 s 净位移均仅约 0.19–0.23 m | 否决默认高速接管 |
| Floor1 0.35 m/s 自动高速任务 | 导航后完成抓取 | 两次在走廊中段进入规划/碰撞恢复失败 | 不采用 |
| Floor1 0.20 m/s 一致速度包络 | 导航、视觉、双指接触和物理抬升全部通过 | `Goal succeeded`、RGB-D 1.33 s、Link7+Link8 fixed joint、cube lift=0.036 m、`PICK_COMPLETE` | 通过（第 1 个干净世界成功回合） |
| Floor1 控制链静态审计 | 明确 actor 与 overlay 边界 | 运动阶段四拍 IK blend=1.0，另有 base force assist | 通过 |
| 历史导航抓取证据复核 | 存在完整成功回合 | Nav2、RGB-D、双指接触、约 43 mm 抬升、载物 1.3 m+ 均有通过记录 | 通过 |
| 前腿拖地动态基线 | 可重复、可量化 | 已完成原始/部分/全覆盖受控测速，尚未补逐腿接触 trace | 部分通过 |
| MuJoCo 0.50 m/s 两模型 A/B | 4000 模型更接近速度目标且稳定 | 4000=`0.4216 m/s`；原始=`0.4077 m/s`；两者均未倒地 | 通过 |
| Gazebo Classic 4000 actor 0.50 m/s | 速度接近 MuJoCo 金标准且姿态稳定 | 暖机后约 `0.486 m/s`，`z_mean≈0.293m`，roll/pitch 峰值较低 | 通过 |
| Gazebo Classic 全接触 Floor1 导航抓取 | 不改变任务状态机且完成抓取 | `Goal succeeded`、双指 fixed joint、抬升 `0.038m`、`PICK_COMPLETE` | 通过 |
| Gazebo Classic 固定降 blend / 分阶段外部切换 | 提高巡航速度且不破坏最后转向 | 最后转向/进度门禁失败 | 不采用 |

### Errors（错误）
| 错误 | 解决方案 |
|-------|------------|
| 新增 WBC 单元测试首次收集失败：宿主未把源码包加入 PYTHONPATH，且外部仓库 pytest 缓存不可写 | 后续设置 PYTHONPATH=src/go2_piper_wbc，并将 cache_dir 指向 /tmp；不重复原命令 |
| WBC 全测试目录首次收集命中 LeggedManip_Lab 中同名 test_piper_kinematics 模块 | 使用 pytest --import-mode=importlib 隔离两个仓库的同名测试，不删除用户缓存 |
| 宿主 pytest 6.2.5 不支持 importlib 隔离，PYTHONPYCACHEPREFIX 也不能解除已注入的同名测试模块 | 停止全目录重试；本次相关测试与 policy_core 单独运行 23/23，通过实际 colcon 构建覆盖安装验证 |
| colcon 默认构建无法在 Go2Arm 工作区 log/ 下创建日志 | 不修改目录权限，改用 /tmp 下独立 build/install/log base |
| 先前启动了错误仓库的键盘控制模型 | 已停止相关容器；后续固定使用 Go2Arm Floor1 入口并启动前校验哈希 |
| 早期分析沿用了 DART“无法站立”的单一路线结论 | 联合读取 Floor1 主计划后，区分 DART 实验失败与 Classic 导航抓取成功主线 |
| 首次补丁同时删除/新增同一路径而被工具拒绝 | 改为逐文件分步重建并成功写入 |
| MuJoCo 原部署脚本依赖宿主 X11/键盘，不能直接无头复用 | 使用同等 contract 的临时无头 rollout 脚本，仅用于只读 A/B，不改项目入口 |
