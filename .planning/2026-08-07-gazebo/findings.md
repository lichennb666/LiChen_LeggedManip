# 发现与决策

## Requirements（需求）
- Gazebo 场景启动即带绿色方块（world 内置，不用延迟 spawn）
- 机械臂启动位姿改为中立位，消除零位→自然位启动扫掠
- 提供 launch + sh 一键启动，在 Gazebo 中对 Go2+Piper 键盘控制运控测试

## Research Findings（研究发现）
- 启动扫掠根因：wbc_node 启动期（startup_delay=3s）目标为 DEFAULT_ANGLES（臂关节全 0），策略激活后把臂拉到自然位；历史 progress 也确认“零关节到策略自然姿态的启动扫掠把方块推下台”。
- 臂自然位（当前配置 pose_command z=0.70、IK 关闭）：来自最近失败 run 的 pregrasp 物理状态 arm=(0.0, 1.24, -0.764, 0.005, -0.391, -0.032)。
- FK 验证：臂基座相对 base (0,0,0.06)，base 高约 0.287m → 臂基座世界 z≈0.347。自然位下 TCP≈(0.396, 0, 0.673)，方块盒 x∈[0.455,0.505], y∈[0.06,0.11], z∈[0.506,0.556] —— 自然位任何连杆都不进入方块盒，安全。
- 当前 spawn 无 -J 参数，Gazebo 中 URDF 关节初始为 0（零位启动）；spawn_entity.py -J joint position 可指定初始关节角。
- **实测发现**：本容器 gazebo_ros (Humble) 的 spawn_entity.py 不支持 -J，也不提供 /gazebo/set_model_configuration 服务。
- **最终方案**：构建时用 `gz sdf -p` 把 URDF 转成 SDF（会丢掉 <ros2_control> 块，脚本需从 URDF 回填），并在每个关节 <axis> 写 <initial_position>；spawn SDF 即带中立位启动。
- pick_transport.world 已有 pickup_table（pose 0.39 0.08 0.496，台面高 0.506）与 drop_table；方块应位于 (0.48, 0.085, 0.531)。
- 任务配置硬编码 target_center_z: 0.531（方块在台面上时正确）；mission 通过 gazebo_ros_state 读实体位姿，world 内置方块即可被读取。
- 失败 run 直接原因：方块在任务开始时已在地面，mission 按台面高度抓取 → 夹空 → PHYSICAL_LIFT_FAILED；15s 延迟 spawn 仍被扫下台，说明启动即自然位 + world 内置方块是正解。
- 容器：osrf/ros:humble-desktop-full；colcon build --symlink-install；setup.py 新入口需重新 build。
- PD 控制器从 policy_contract.yaml 加载 18 关节 + joint7/joint8 抓爪（默认 [0.04,-0.04]），抓爪走 /go2_piper/gripper/target。
- teleop 采用两段式 sh：后台启动场景 launch（-d 容器），前台 termios 键盘节点发布 /cmd_vel（command_timeout=0.2s，需持续发布）；就绪信号用 /go2_piper/wbc/status 的 "ready": true。

## Technical Decisions（技术决策）
| 决策 | 理由 |
|----------|-----------|
| spawn SDF（<initial_position> 写入 20 关节：腿=DEFAULT_ANGLES，臂=自然位），wbc 新增 startup_target 参数在启动期保持该位姿 | 消除零位→自然位扫掠，臂在自然位不碰方块；Humble spawn_entity 无 -J |
| world 内置 target_cube（内联模型定义） | 场景启动即有方块，不依赖模型路径 |
| 自写 teleop_keyboard 节点 + run_teleop_test.sh 两段式启动 | launch 内节点无法可靠读终端 stdin |

## Issues Encountered（遇到的问题）
| 问题 | 解决方案 |
|-------|------------|
| 启动期传感器未就绪分支仍发布 DEFAULT_ANGLES，可能短暂拉臂回零位 | 该分支也改发 startup_target |

## Resources（资源）
- logs/sim2sim/gazebo_aligned_natural_pose.jsonl（旧对齐自然位，pose z=0.5）
- ros2_ws/src/go2_piper_wbc/config/policy_contract.yaml（DEFAULT_ANGLES）
- ros2_ws/src/go2_piper_wbc/go2_piper_wbc/piper_kinematics.py（FK/TCP）
- 旧计划 .planning/2026-08-05-go2-piper-ros2-wbc-grasp-transport/（扫掠问题历史轮次）

## 2026-08-10 当前复跑发现
- 当前 `GUI=false ./docker/run_demo.sh` 可正常启动 Gazebo、robot_state_publisher、controllers、WBC、color_detector 和 mission；不是 GUI/模型加载层面的失败。
- 第一次 demo：检测目标约 `(0.481,0.148,0.531)`，descend 后夹爪闭合，但 lift 阶段 pad z 无法收敛到 `lift_start_pad + 0.06`，最终 `VISUAL_SERVO_FAILED:lift,error=(0.000,0.000,0.037)`。过程中 base y 从约 0.230 漂到 0.312，pad x/y 也有明显漂移；方块 z 只从 0.531 到约 0.533，不是有效抬升。
- 第二次 demo：mission 开始前方块已在地面，`cube before detection=(0.635,0.125,0.025)`；检测仍把目标 z 强制为 `target_center_z=0.531`，导致后续视觉伺服追台面高度的虚假目标并在 grasp 阶段失败。
- WBC response gate：`linear.x=0.2` 持续 5s 时，稳定性门禁通过（无高度/姿态超限），但响应指标为 `response_dx=0.039m`, `response_dy=-0.072m`, `response_dyaw=0.082rad`。说明当前策略在 Gazebo 中低速 x 指令跟踪很弱且横漂，不能仅凭稳定性 PASS 认为运输可用。
- 安装后的 SDF 生成存在结构错误：20 个 `<initial_position>` 堆叠到 `FL_hip_joint` 的 `<axis>` 下，而不是分别写入对应物理 joint；同时 `ros2_control` 中 Piper `joint1` 到 `joint6` 的 `initial_value` 仍来自 `policy_contract.yaml` 的零默认角。当前“机械臂中立位 spawn”不可信。

## 2026-08-10 后续修复发现
- `generate_sdf.py` 已改为 XML 解析方式给 20 个物理 joint 分别写入 `<initial_position>`；复查生成 SDF：物理 joint 数 20、initial_position 数 20，不再堆叠到 `FL_hip_joint`。
- `convert_legacy_urdf.py` 已从 `startup_pose.json` 回填 `ros2_control` 的 Piper/夹爪 initial_value；Gazebo 启动日志显示 `joint2=1.24`, `joint3=-0.764`, `joint7=0.04`, `joint8=-0.04`。
- `wbc_node.py` 增加 `hold_idle_arm_startup_target=True` 后，无任务场景 24s 内方块不再掉到地面；剩余为厘米级台面滑动。
- 夹爪几何诊断显示：修正 `joint8` 后，若 Link8 collision pad 仍使用本地 `0 +0.04 0`，则 Link7/Link8 pad 在 Link6 下上下错开 8cm，闭合时仍不是对夹。修正 Link8 pad 为本地 `0 -0.04 0` 后，open 状态两个 pad 同高、间距 8cm，closed 状态同高重合。
- 夹爪 collision 原 `0.05 0.05 0.015` 在夹持方向只有 15mm 厚，闭合后夹爪关节仍接近开口且方块不随 lift；已加厚到 `0.05 0.05 0.035`，不增加接近/竖直方向尺寸，避免重新刮桌。
- 物理抓取模式仍不稳定：`grasp_forward_offset=-0.02` 时 pad 容易闭在方块后方；`+0.03` 时 descend 明显推方块前移。当前默认回到 `0.0`。真实物理模式后续还需调接触、偏置、Piper IK 和 WBC 站立漂移。
- 默认演示模式改为 `PHYSICAL_GRASP=false`，自动使用 `go2_piper_demo.sdf` 加载 demo-only virtual grasp 插件；`PHYSICAL_GRASP=true` 保留纯物理 SDF/接触路径。
- 默认演示复跑通过：`GUI=false ./docker/run_demo.sh` 加载 `go2_piper_demo.sdf`，日志出现 `Go2-Piper demo virtual-grasp plugin ready`，最终 `DONE progress=100%`、`pick and transport complete`，运输距离验证 `0.210m`。

## 2026-08-10 物理抓取专项进展
- 收紧物理 PREGRASP 门限后，原先 0.15m `pregrasp_height` 被证明不可达：pad z 长时间停在约 0.54m，目标误差约 0.16m，最终 `VISUAL_SERVO_FAILED:pregrasp`。这解释了之前为什么 0.25m 大容差会“假通过”。
- 将 `pregrasp_height` 降到 0.04m 后，pregrasp 可收敛，但暴露出 pad 前后 footprint 太大：方块在 PREGRASP/DESCEND 阶段被推走。
- 将 finger pad box 从 `0.05 0.05 0.035` 调整为 `0.025 0.05 0.035` 后，纯物理模式第一次通过 lift 验证：`physical grasp verified: cube lift=0.029m`。
- 后续运输仍失败：`PHYSICAL_TRANSPORT_FAILED:offset=0.068m,aperture=0.001m,z=0.531m`，说明方块在携物行走中滑落回桌面。
- 尝试增强夹爪保持力（stiffness/effort 提高、close wait 1.5s）是负向结果：夹爪过早/过强闭合，方块被挤出，最终 `PHYSICAL_LIFT_FAILED`。该实验已回退。
- 默认 demo 回归仍通过：`GUI=false ./docker/run_demo.sh` 最终 `DONE progress=100%`、`pick and transport complete`，运输距离验证 `0.200m`。

## 2026-08-10 夹爪 URDF 与 WBC 控制链路判断
- 生成脚本和生成后的 SDF 均显示 Link7/Link8 是镜像夹爪：`joint7` pose 为 `+1.5708`、axis `0 0 -1`、初始 `0.04`；`joint8` pose 为 `-1.5708`、axis `0 0 1`、初始 `-0.04`。Link7 pad 本地 `0 +0.04 0`，Link8 pad 本地 `0 -0.04 0`。因此仿真中“一边看起来反过来”更像是左右镜像的正常表现，不是当前主要故障点。
- 当前可疑点不是夹爪 URDF 方向，而是纯物理运输阶段接触保持不稳：已实测过大夹爪力会把方块挤出，说明不应继续通过简单加力解决。
- WBC 节点是 18 维策略适配器：12 个 Go2 腿关节 + Piper `joint1`~`joint6`。末端目标通过 `/go2_piper/wbc/ee_target` 触发 Piper 位置 IK 覆盖 arm 6 轴；夹爪 `joint7/joint8` 走独立 `/go2_piper/gripper/target` 和 PD 控制，不属于 WBC policy action。
- 因此后续不建议“重新微调夹爪/WBC 模型”作为第一优先级。更直接的方向是调 mission 层的抓取/携物姿态、运输速度扰动和接触几何；只有在底盘携物行走仍系统性漂移/抖动时，才进入 WBC/策略微调。

## 2026-08-10 物理模式追加复测
- 保留的最小改动：携物阶段硬编码 `0.20m/s` 改为参数 `carrying_min_speed=0.18`，并将 `mission.yaml` 的 `carrying_linear_speed` 从 `0.25` 降到 `0.20`。这只影响运输阶段，不改变夹爪/WBC 模型。
- 物理复测未到运输阶段：方块在 PREGRASP/DESCEND/CLOSE 被继续向 +x 推移，最终 `PHYSICAL_LIFT_FAILED`。典型日志：检测时 cube x≈0.558，pregrasp 后 x≈0.604，closed 后 x≈0.614，lift 后 cube z 仍≈0.531。
- 诊断性收紧 PREGRASP 垂直容差到 0.02m 后，任务在 PREGRASP 超时：pad z 误差长期约 0.037~0.041m，说明当前 WBC+IK 在平面贴近方块时无法稳定达到“方块中心+4cm”的预抓高度；该诊断性改动已回退。
- 尝试在 PREGRASP 后刷新 Gazebo 实体目标、并将物理 `grasp_height_offset` 提到 0.02m 是负向结果：方块前推更严重（closed 后 x≈0.666），已回退。
- 结论更新：纯物理当前主瓶颈已经从“运输滑落”回到“接近/闭合阶段前向推挤不稳定”。继续调运输前，应先解决接近时 pad 实际高度/碰撞接触导致的推块问题。
- 默认 demo 回归通过：`GUI=false ./docker/run_demo.sh` 加载 `go2_piper_demo.sdf` 和 virtual-grasp plugin，最终 `DONE progress=100%`、`pick and transport complete`，运输距离 `0.202m`。

## 2026-08-10 追加判断：夹爪 URDF 与 WBC 是否需要再微调
- Link7/Link8 的 URDF/SDF 方向目前是镜像夹爪关系，不是同向复制：Link7 pad 本地 `0 +0.04 0`，Link8 pad 本地 `0 -0.04 0`。因此 Gazebo 里“一边看起来反过来”本身不构成错误；如果两指闭合后 pad 同高、间距随 joint7/joint8 对称变化，就是正确表现。
- 夹爪 URDF 仍有一个和任务有关的工程问题：finger pad 接近方向 collision footprint 会影响纯物理接近阶段是否推方块。将 box 从 `0.025 0.05 0.035` 缩小到 `0.015 0.05 0.035` 后，推块减轻，并出现一次 `physical grasp verified: cube lift=0.036m`，但运输仍滑落。
- 收紧 PREGRASP/DESCEND 平面门限到 `0.015` 是负向结果：当前 WBC+IK 不能稳定达到该门限，已回退到 `0.025`。这说明问题不是简单“对准精度不够”，而是低位接近时底盘/臂/接触耦合导致的可达性和推挤。
- PREGRASP 后刷新 Gazebo 目标也是负向结果：会让 DESCEND/grasp 发散，已回退。
- 当前 WBC 策略本体不建议继续作为第一优先级微调。原因：默认 demo 已通过；纯物理模式的最新失败集中在夹爪接触几何、低位接近姿态、携物横向扰动。WBC 需要关注的是底盘低速/横漂/携物稳定，但在接近接触未稳定前，直接再次微调策略很难定位收益。
- 保留的最小运输侧改动：携物阶段 `cmd.linear.y = 0.0`，减少方块夹持边缘状态下的横向扰动；该改动尚未在完整纯物理运输中验证通过，因为后续 run 多数未稳定进入运输阶段。
- 默认 demo 最新回归仍通过：`GUI=false ./docker/run_demo.sh` 最终 `DONE progress=100%`、`pick and transport complete`、`base transport verified: distance=0.201m`。

## 2026-08-10 纯物理启动漂移与接近实验结论
- 物理模式 mission 17s 启动延迟会给轻质 cube 留出明显漂移时间；将 `PHYSICAL_GRASP=true` 分支的 mission 启动延迟缩到 10s 是正向改动，`cube before detection` 从 y≈0.24 级降到 y≈0.16 级。默认 demo 仍使用 17s，不受影响。
- finger pad 第二尺寸从 `0.05` 缩到 `0.035/0.04` 都是负向结果：`0.035` 推块变小但 lift 失败，`0.04` 仍 lift 失败且 y 向推块严重。最终回到 `0.015 0.05 0.035`。
- PREGRASP 平面门限放宽到 `0.04` 虽然能避免 pregrasp 超时，但会过早进入 DESCEND，导致方块 x 明显被推走；该方向已回退。
- `grasp_height_offset=0.015` 不能形成有效夹持，lift 只有 4mm；已回退到 `0.005`。
- 给 pickup/drop table 显式高摩擦 `mu=10` 是负向结果：启动漂移更糟，方块跑出检测区域并 `DETECTION_TIMEOUT`；已回退。
- 默认 demo 回归继续通过：`DONE progress=100%`、`pick and transport complete`、`base transport verified: distance=0.209m`。

## 2026-08-10 物理 reset 后的当前判断
- 任务开始时主动 reset 方块是正向工程改动：它把纯物理模式从“启动后方块随机漂移/掉落”收敛成“固定初始姿态下接近/闭合仍推块”的可复现问题。
- 当前保留调试基准为 `cube_reset_pose: [0.50, 0.16, 0.531]`。`[0.48,0.18,0.531]` 出现检测超时；`[0.54,0.16,0.531]` 能检测但接近时明显从后方把方块推到 x≈0.60。
- `grasp_forward_offset=-0.015` 和 `-0.005` 均为负向：后撤会减少正向推块，但同时导致夹爪错过方块或产生 y 向推挤；当前不继续沿负 offset 调。
- `pregrasp_height=0.06` 也是负向：当前 WBC+Piper IK 在该贴近位置无法稳定达到更高预抓取高度，PREGRASP 超时；当前保留 `0.04`。
- 夹爪 URDF 方向仍不是首要问题：Link7/Link8 镜像关系保留，finger pad collision 当前为 `0.015 0.05 0.035`。视觉上“一边反过来”与镜像夹爪一致；真正影响任务的是接近方向 footprint 和低位接触时序。
- WBC 不建议现在继续微调：默认 demo 已通过，纯物理最新失败集中在夹爪接触/低位接近/闭合时序。只有当接近阶段能稳定 lift 后，若运输仍系统性横漂或滑落，再评估 WBC/策略微调。
- 默认 demo 最新回归通过：`GUI=false ./docker/run_demo.sh` 最终 `DONE progress=100%`、`pick and transport complete`。

## 2026-08-10 lift 横向锁定实验结论
- `PRECLOSE` 是负向：预闭合目标 `[0.028,-0.028]` 在短等待内没有让夹爪实际收到位，反而在方块附近增加 y 向接触扰动；该方案已移除。
- `vertical_only=True` 原实现会把当前 TCP x/y 作为每一轮新命令，因此 lift 过程中一旦 IK/底盘横漂，目标也会跟着漂，导致横向拖拽。锁定进入 lift 时的 TCP x/y 后，纯物理复测中 pad x/y 漂移从上一次 x≈0.35 级改善到 x≈0.47~0.49 级，是正向修正。
- lift 横向锁定没有解决根因：在 reset `[0.50,0.16]` 下，pregrasp 仍可能将 cube 从 x≈0.507 推到 x≈0.569，后续 close/lift 是在追旧目标；最终 z 仍无法抬起。
- reset `[0.46,0.16]` 是负向：虽然 PREGRASP 更快满足，但 close 后 cube 被带到 x≈0.431，lift z 误差更大；已回退。
- 当前保留的最小正向代码改动是 lift 锁定 TCP x/y。下一步真正要处理的是 PREGRASP/DESCEND 阶段方块被推离目标，而不是继续优化 lift。

## 2026-08-10 物理目标校正与门限复查
- 纯物理目标使用 Gazebo truth 是阶段性正向：检测仍由 RGB-D 触发，但目标 x/y/z 改用实体中心，避免 1~2cm 感知偏差主导接触实验。默认 demo 不受影响。
- Gazebo truth 校正后，PREGRASP 推块较轻，但 CLOSE 仍会把方块推出。例如一轮中 cube before approach 约 `(0.490,0.137)`，pregrasp 后约 `(0.493,0.142)`，descend 后约 `(0.496,0.144)`，close 后变成 `(0.521,0.163)`，说明剩余问题集中在最终闭合接触。
- 单独将 DESCEND/grasp 平面门限收紧到 0.015 是负向：没有改善 close，反而在一次 run 中 PREGRASP 超时，另一次 run 中 DESCEND/close 把方块推得更远；已回退到 0.025。
- 当前不应继续通过视觉伺服门限微调解决问题。下一步更直接的是处理 close 时夹爪与方块的接触关系：例如闭合前的相对位置、闭合速度/时长、夹持高度或 pad 有效接触面，而不是继续压缩 x/y tolerance。

## 2026-08-11 低预抓取与物理目标漂移补偿结论
- `pregrasp_height=0.04` 在当前 WBC+Piper IK 下处于可达边界：两轮物理测试都在 PREGRASP 卡住，pad z 误差约 `0.041m`，没有机会验证 close。降到 `0.035` 后，PREGRASP 能快速通过，默认 demo 也保持通过。
- 纯物理单独调夹爪闭合目标不是充分解法：`[0.025,-0.025]` 推块较少但夹持不足；`[0.022,-0.022]` 能让关节实际闭到目标附近，但如果目标已被 PREGRASP 推走，仍会闭在方块后侧。
- PREGRASP 后的方块漂移是当前关键中间变量。一次正向 run 中，目标刷新日志为 `physical target refreshed after pregrasp: drift=0.036m`，随后纯物理抓取首次在本轮通过 lift：`physical grasp verified: cube lift=0.025m`。
- 目标刷新必须窄条件触发：只在 `PHYSICAL_GRASP=true` 且 PREGRASP 后 Gazebo 实体 x/y 漂移超过 `0.02m` 时刷新一次。它不同于此前无条件/高位组合刷新实验，当前用于补偿已观测到的开口指垫推块。
- 纯物理当前已从“完全抬不起”推进到“可偶发 lift，但运输保持失败”。最新运输失败为 `PHYSICAL_TRANSPORT_FAILED:offset=0.183m,aperture=0.057m,z=0.549m`：高度仍在桌面以上，主要问题是方块相对 TCP 偏移扩大。
- 将携物速度降到 `0.10/0.08` 不破坏默认 demo，默认 demo 仍 `DONE progress=100%`，运输距离 `0.203m`。但该降速在纯物理中尚未验证到运输阶段，因为后续 run 又失败在 lift。
- 现阶段不应继续盲目加深闭合到 `[0,0]` 或继续收紧视觉伺服门限；更直接的下一步是稳定 lift 后的携物姿态/运输保持，或改进 close 前的接触包络，使方块在两指之间而不是侧边/后方。

## 2026-08-11 最新复跑：默认通过，纯物理仍卡 lift
- `PHYSICAL_GRASP=true GUI=false ./docker/run_demo.sh` 在 `LIFT_STOW` 失败：pad z 误差从进入 lift 起长期保持约 `0.035m`，最终 `VISUAL_SERVO_FAILED:lift,error=(0.000,0.000,0.035)`。
- 这轮失败没有进入 `DRIVE_DROP`，因此 `maximum_transport_offset=0.20` 尚未被纯物理运输阶段验证。
- WBC 状态显示 `ik_active=True` 且 `ik_residual` 接近 0，说明不是 IK 数值求解没找到目标；更可能是 Gazebo 实体 TF/实际关节响应/夹爪与桌面或方块接触导致末端 pad 没有实际抬升。
- `GUI=false ./docker/run_demo.sh` 默认 demo 最新回归通过：`DONE progress=100%`、`pick and transport complete`、运输距离 `0.203m`。所以当前问题限定在 `PHYSICAL_GRASP=true` 的纯物理接触/lift 路径，不是默认任务编排或 WBC 全局失效。

## 2026-08-11 lift 诊断与接近阶段修正
- 新增的物理诊断日志确认：lift 阶段 `cmd.z` 已持续发布，`arm_ik_active=True`，`arm_ik_residual≈0`，arm 关节也在动；失败不是“命令没发”或“IK 没解”。
- TCP 开环 lift 是负向：在未夹稳时会让 TCP/arm 偏离，方块 lift 为 `-0.002m`，已回退到 pad visual servo lift。
- PREGRASP 的 `vertical_tolerance_override=0.04` 会允许 pad 比目标低约 3.5~4cm 时通过，容易低位擦块；收紧到 `0.025` 后不再低位假通过。
- 单独收紧 z 容差会暴露底盘零速漂移：PREGRASP 期间 base y 漂移可把 pad y 误差越拉越大。直接 `locomotion_enabled=false` 是负向，会导致 base 高度掉到约 0.13m。
- 更稳的做法是 physical PREGRASP/DESCEND 使用已有 station keeping：实测接近阶段快速通过，方块未再大幅推走。默认 demo 回归仍通过，运输距离 `0.202m`。
- `gripper_close_wait=1.2` 是负向：close 后夹爪仍接近开口，且方块被推走；说明不是简单等待不足，而是 close 接触姿态/一侧指爪顶块导致，已回退到 `0.75`。
- 当前剩余瓶颈重新收敛到 lift/夹持保持：接近阶段改善后，纯物理仍 `VISUAL_SERVO_FAILED:lift`，需要下一步针对 close 前夹爪相对方块的 y 包络或 lift 时保持姿态处理。

## 2026-08-11 lateral offset 复测结论
- `physical_grasp_lateral_offset=0.012` 是负向实验：纯物理 run 未进入 close，PREGRASP 长期卡在 pad z 低约 3.5cm，最终 `VISUAL_SERVO_FAILED:pregrasp,error=(0.014,-0.005,0.035)`。
- 该结果说明单纯把目标向 +y 偏 1.2cm 不能解决“方块靠近单侧指爪”的问题，反而破坏 PREGRASP 收敛；已回退到 `physical_grasp_lateral_offset=0.0`。
- 默认 demo 在回退后仍通过：`DONE progress=100%`、`pick and transport complete`、运输距离 `0.204m`。当前可交付路径仍稳定，问题限定在 `PHYSICAL_GRASP=true` 纯物理接触路径。

## 2026-08-11 reset 清速度与预抓高度复测结论
- `SetEntityState` reset 方块只设置 pose、不清 twist 会保留 Gazebo 实体旧速度；这解释了 reset 后方块仍快速漂移的现象。清零 `twist.linear/angular` 后，`cube before detection` 从 x≈0.545 改善到 x≈0.506，接近设定 `[0.50,0.16,0.531]`。
- `pregrasp_height=0.025` 和 `0.030` 都不是解法：它们能让 PREGRASP 更容易通过，但会让开口指垫/低位接近把 cube 向 +x 推走，最终仍 lift 失败。
- 当前应保留 reset 清速度；不应继续靠降低 pregrasp 高度解决。剩余问题更像接近方向 collision/指垫包络或夹爪姿态导致的前向推块。
- 默认 demo 在最终保留改动后仍通过：`DONE progress=100%`、`pick and transport complete`、运输距离 `0.202m`。

## 2026-08-11 PREGRASP-only x 后撤验证结论
- 只在 PREGRASP 阶段使用 `physical_pregrasp_forward_offset=-0.02` 是负向结果：物理 run 没有进入 DESCEND/CLOSE，PREGRASP 期间 pad z 误差长期约 `0.038m`，最终 `VISUAL_SERVO_FAILED:pregrasp,error=(0.005,0.008,0.038)`。
- 该结果说明“预抓阶段单独后撤”不能解决当前接触问题，反而使已知的 PREGRASP z 可达性瓶颈重新出现。它不应作为后续方向继续微调。
- 参数已回到 `0.0`，默认 demo 回归通过：`DONE progress=100%`、`pick and transport complete`、运输距离 `0.203m`。
- 默认 demo 完成后 shutdown 阶段 `wbc_node` 出现一次 rclpy `RuntimeError`，但发生在任务成功和 required process 触发系统关闭之后，暂归类为非阻塞清理问题。

## 2026-08-11 finger pad 尺寸实验结论
- finger pad 第一维从 `0.015` 收窄到 `0.010` 后，PREGRASP/DESCEND 能快速通过，说明边缘擦碰有所减轻；但 close 后仍不能稳定 lift，最终 `VISUAL_SERVO_FAILED:lift,error=(0.000,0.000,0.030)`。这不是可保留解。
- finger pad 第三维从 `0.035` 增加到 `0.045` 是负向：close 后夹爪仍接近开口，lift z 误差扩大到约 `0.045m`。这说明不能通过简单增大 pad 高度解决夹持。
- 两项实验均已回退到 `0.015 0.05 0.035`。默认 demo 回归通过，运输距离 `0.203m`。
- 新结论：当前不应继续盲目调 pad 尺寸。更直接的下一步是诊断 CLOSE 后 cube 相对 Link7/Link8 pad 的位置和夹爪实际接触状态，判断是“夹在侧边/后方”还是“接触后摩擦/约束不足”。

## 2026-08-11 CLOSE 几何诊断与 DESCEND z 容差结论
- baseline 纯物理失败时，新增 close 后几何日志显示 cube 沿手指轴方向基本正确，但相对两指中心线偏差较大：`off_finger_line≈0.031m`，`between_margin≈0.018m`，pad distance 约 `0.075m`。这说明当时方块更像贴在指爪侧边/边缘，而不是干净进入两指有效夹持中心。
- 将 physical DESCEND 的 z 容差从 `0.02` 收紧到 `0.01` 是局部正向：复测中 DESCEND 后 `off_finger_line≈0.011m`，CLOSE 后 `off_finger_line≈0.010m`，几何对中明显好于 baseline。
- 该改动没有解决纯物理 lift：复测仍在 `LIFT_STOW` 失败，最终 `VISUAL_SERVO_FAILED:lift,error=(0.000,0.000,0.048)`。lift 阶段 pad z 实体高度没有跟随目标上升，close 后 gripper 状态仍发生变化，说明剩余瓶颈不是单纯几何对准，而是接触保持/夹爪响应/实体约束问题。
- 默认 demo 对该改动不敏感，最新回归通过：`DONE progress=100%`、`pick and transport complete`、`base transport verified: distance=0.203m`。
- 当前保留判断：夹爪 URDF 镜像方向不是首要问题；WBC 也不是下一步优先微调对象。更直接的下一步应检查 close 后 Link7/Link8 与 cube 是否发生有效接触、夹爪关节为何在 lift 中重新打开/漂移，以及 Gazebo 接触参数或夹爪控制器是否给了足够保持约束。

## 2026-08-11 夹爪 effort 与 lift 阶段新证据
- `cube_reset_pose` 从 `[0.50,0.16,0.531]` 调到 `[0.48,0.16,0.531]` 是当前正向：一轮中 reset 后 cube before detection≈`(0.478,0.148,0.532)`，可以稳定进入 CLOSE；原 x=0.50 在同一代码状态下会快速漂到 x≈0.53 并导致 PREGRASP 超时。
- CLOSE 后几何已经可以非常好：一轮中 `grasp geometry closed` 为 `off_finger_line≈0.002m`，说明方块基本位于两指有效中心线，不再是明显夹偏。
- 夹爪 effort 诊断显示 joint7/joint8 没有撞到 effort limit：CLOSE/closing 期间 effort 约 `1~4`，更深闭合时约 `5`，远低于 controller 的 `20`。因此“effort limit 饱和导致夹不住”不是当前主因。
- 更深 `physical_gripper_closed=[0.015,-0.015]` 是负向/未达标：后期夹爪确实能接近 `[0.015,-0.015]`，但 pad z 仍不跟随 lift 目标上升，最终 `VISUAL_SERVO_FAILED:lift,error≈0.033m`。
- `grasp_height_offset=0.010` 是负向：还没进入 DESCEND/CLOSE，就在 PREGRASP 因 z 误差约 `0.037m` 超时。此前 `0.015` 也已是负向，因此当前保留 `0.005`。
- 当前结论进一步收敛：纯物理失败不应继续从夹爪镜像、pad 尺寸、闭合目标、抓取高度或 WBC 微调入手。下一步应直接看 lift 时 Gazebo 接触/约束：例如 Link6/Link7/Link8 是否被 cube/table 接触卡住，或者 arm IK 命令虽然解算成功但实体关节在接触约束下没有产生预期 pad z 位移。
