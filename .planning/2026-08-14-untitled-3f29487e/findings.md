# 发现与决策

## Requirements（需求）
- 保留 ROS 2 Humble、Piper 和 WBC。
- 默认 Gazebo 仅保留地面、机器人和一个可抓物体，不再保留原桌子。
- 默认流程删除视觉定位和自动取放任务，改为键盘移动机器狗与手动抓取。
- 辅助抓取继续采用 Go2Arm_sim2sim 风格：双指接触后触发 fixed joint。

## Research Findings（研究发现）
- 实际工程为 `/home/lili/LeggedManip_Lab/ros2_ws`，当前有 8 个 ROS 2 包。
- 旧 `grasp_plugin.cpp` 已检查 Link7/Link8 接触，但无接触时也返回成功，随后每个物理步用 `SetWorldPose` 搬运 `target_cube`，不是固定关节。
- 旧 mission 在夹爪闭合前调用 attach，之后 assisted lift 仍由插件增加 z 偏移，运输还可切到 mission 侧 `SetEntityState`。
- Gazebo Classic 支持运行时由 physics engine 创建 `fixed` joint，并通过 `Attach/Load/Init` 连接机器人 link 与外部模型 link；释放时可 `Detach/Fini`。
- Gazebo ContactManager 默认可能丢弃无订阅者的 contact；插件需设置 `SetNeverDropContacts(true)`，服务才能稳定读取双指接触快照。
- 本地没有名为 Go2Arm_sim2sim 的仓库；公开检索未找到精确项目，但双指接触后建立固定约束的目标语义明确。
- 运行时实测插件成功加载为 `Link7 + Link8 -> target_box::handle`，无接触调用 attach 返回 `success=False` 且分别报告左右接触状态。
- 用户复核后认为视觉桌面任务仍接近旧效果，明确要求回到开源 sim2sim 的最小手动交互思路。
- 现有 `teleop_test.launch.py` 只启动仿真、WBC 和里程计，不启动 perception/mission，是新默认入口的合适基线。
- 现有 `teleop_keyboard.py` 只有 `/cmd_vel`；需要增加 `/go2_piper/gripper/target` 和 attach/detach 服务控制。
- Gazebo `ContactManager::GetContacts()` 的有效内容只存在于物理步的短暂阶段，ROS 服务回调直接读取会漏检；改用 `CreateFilter` 的 transport 接触主题后，真实双指接触验收返回 `success=True`。
- 学习式 WBC 的自由站立在多次无头压力测试中可能发生较大漂移，因此默认交互增加 I/K/J/L/U/O 末端微调和 F 附着重试，不再依赖一次性绝对预对齐。
- 首两次默认 WBC/IK 端到端试跑均完成绿色把手检测，但接近动作会推动早期短间隙箱体并在闭爪前失败；最终模型把把手与主体间隙从 30 mm 加到 80 mm、主体质量从 80 g 加到 200 g，并让辅助路径持续跟踪把手。最终结构重新通过构建、SDF 和回归验证，但未再宣称完整运输 DONE。

## Technical Decisions（技术决策）
| 决策 | 理由 |
|----------|-----------|
| 绿色只用于前置把手握杆 | 保持 HSV 参数与检测节点不变，并让检测点自然对应抓取点 |
| fixed joint 连接 Link6 与箱体 handle link | Link7/8 负责接触判定，Link6 提供稳定父 link，避免约束直接挂到滑动手指 |

## Issues Encountered（遇到的问题）
| 问题 | 解决方案 |
|-------|------------|
| 全主目录关键词搜索进入 npm/微信缓存并产生超大输出 | 后续仅在 `LeggedManip_Lab` 的源码、配置与模型目录内检索 |
| 首两次端到端运行中 WBC/IK 接近会把短间隙箱体扫离台面 | 加长把手净空、增加箱体质量、动态跟踪把手；最终安全成败仍交给双指接触门禁 |

## Resources（资源）
- `ros2_ws/src/go2_piper_gazebo_plugins/src/grasp_plugin.cpp`
- `ros2_ws/src/go2_piper_bringup/worlds/pick_transport.world`
- `ros2_ws/src/go2_piper_mission/go2_piper_mission/mission_server.py`
