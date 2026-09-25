# 发现与决策

## Requirements（需求）
- ROS 2 Humble、Gazebo Classic、现有 RL-WBC、HSV+RGB-D、固定取放点、接触后附着、保留真机迁移边界。

## Research Findings（研究发现）
- 宿主机为 Ubuntu 20.04.6，未安装 `/opt/ros/humble`；Docker 28.1.1 可用。
- `GO2-PIPER-WBC` 策略文件存在，大小约 1.1 MB；动作是 18 维腿臂关节位置目标，夹爪不在策略动作中。
- 策略观测为每帧 70 维、3 帧历史共 210 维；末端指令为 `(x_base, y_base, z_world, qw_base, qx_base, qy_base, qz_base)`。
- 现有 ROS1 Gazebo 原型使用 fake MoveIt controller、Gazebo 真值检测和不完整运输状态机，不能直接复用为 ROS 2 任务。
- ROS1 Go2+Piper URDF 已包含完整惯量、mesh、D435、IMU 和20个可控关节，可通过构建期转换保留结构并替换插件/控制标签。
- 现有所谓 WBC 是 210D 历史观测到 18D 关节位置目标的学习策略，再由 PD 输出力矩；不是 QP/逆动力学 WBC。
- 未加适配器时，基于真实 `/odom` 的 60 秒门控能正确检测到横滚超限并失败；加入演示平面稳定器后通过。
- 当前主机上 Gazebo D435 能加载但 RGB/depth 端点不稳定，合成 RGB-D 能稳定驱动同一 HSV/深度/TF 检测代码。
- 现有策略未在 Gazebo Classic 接触域训练，机械臂腕部无法可靠接近方块，因此最终 Demo 对持物采用明确的虚拟耦合，不能用于物理抓取结论。
- GUI 黑屏时 `gzclient` 已连接 Gazebo master，但日志停在 `Waiting for model database update to complete...`；容器无法访问在线模型库导致视图区不初始化。
- 禁用在线模型数据库并使用 `/usr/share/gazebo-11/models` 后，截图确认视图区恢复，Sim Time 正常增长且约 33 FPS；OGRE 当前仍使用 llvmpipe 软件渲染。
- 原转换器把完整 Go2 DAE 替换为 MuJoCo 拆分资源中的第一片，例如仅使用 `base_0.stl`，而完整机身需要 `base_0..4`；Piper 也丢失了完整 DAE 外观。
- Gazebo 将 URDF 的 `package://` mesh 转为 `model://` 查找；`GAZEBO_MODEL_PATH` 必须包含 `ros2_ws/install/go2_piper_description/share`。

## Sim2Sim 控制审计（2026-08-06）

### 新确认的高优先级控制契约问题

- **ROS 默认髋关节角符号与训练配置相反**。训练配置按关节名定义为
  `FL/RL=+0.1 rad`、`FR/RR=-0.1 rad`；策略动作顺序是
  `FR, FL, RR, RL`，因此策略空间中的默认向量应为
  `[-0.1, ..., +0.1, ..., -0.1, ..., +0.1, ...]`。当前
  `wbc_node.py` 使用了完全相反的符号。这不仅改变站姿，
  还会让观测量 `q-default_q` 和动作目标 `default_q + action*scale` 同时产生系统偏置。
- MuJoCo 的 `qpos` 原始腿顺序是 `FL,FR,RL,RR`；其 `default_angles` 数组虽然注释误写为
  策略顺序，实际按原始顺序使用后恰好得到训练站姿。但策略输出是 `FR,FL,RR,RL`，当前
  直接加到原始顺序的 `target_dof_pos`，缺少策略到 MuJoCo 的逆映射。应将“仿真原始顺序、
  策略顺序、默认角、动作目标”封装在同一适配器内，并通过带唯一哨兵值的双向映射测试，
  禁止靠注释和位置碰巧对齐。
- **内环 PD 频率错误**。训练为 `sim.dt=0.005 s`，策略 decimation=4，即物理/PD
  200 Hz、策略 50 Hz；MuJoCo 部署同样在 200 Hz 物理步内执行 PD。当前 ROS 节点把
  推理与 PD 绑在同一个 50 Hz 定时器，力矩随后由 effort controller 零阶保持；Gazebo
  与 controller manager 虽运行在 500 Hz，却没有在该频率重新计算 PD。正确结构应拆成
  “50 Hz 策略目标 + 200 Hz（或经验证的更高频）关节反馈 PD”。
- **Gazebo 腿部碰撞几何缺失**。遗留 URDF 中 `*_calflower`、`*_calflower1`、
  `*_foot` 是无 joint 连接的孤立 link，转换脚本会将其裁掉；生成模型因此只有小腿上段
  圆柱，没有下段圆柱和足端球。MuJoCo 每条腿都有两段小腿碰撞体和半径 0.022 m 的足球。
  这会直接改变触地点、支撑多边形、法向力和摩擦力矩，必须先修复再谈增益。
- **动作裁剪不一致**。训练环境动作裁剪是 `[-10, 10]`，当前 ROS 和 MuJoCo 部署使用
  `[-20, 20]`。应以训练导出契约为准改为 `[-10, 10]`，并记录饱和统计。
- **末端默认命令曾被错误改成“指向目标”姿态**。复查训练 `WBCCommandsCfg` 与
  `UniformPoseWBCCommand` 后确认：该权重训练初始范围的 roll/pitch/yaw 全为 0，命令
  四元数应为 `[1,0,0,0]`（wxyz）；位置是 `x/y` 位于 link0 frame、`z` 位于 world
  frame。部署端的指向目标公式导致首帧 `joint5` action 从约 `+0.28` 变成 `-2.48`，
  已改回训练中性姿态 `[0.425,0,0.5,1,0,0,0]`。
- 当前 ROS 命令转换使用 `/odom` 的 base 位姿。由于当前安装固定变换只有 z 偏移，XY 与
  姿态数值碰巧一致，但实现仍应从 TF 获取 `link0`（Gazebo 模型中需把 `piper_l_base`
  统一命名/别名为 `link0`），避免模型更新后静默破坏命令契约。

### 动力学与执行器差异

- 训练执行器：腿 `Kp=30, Kd=0.6`；臂
  `Kp=[50,50,80,30,30,20]`、`Kd=[3,2,3,3,2.5,1]`，并包含
  `armature=0.01`、`friction=0.01` 和 0--4 个控制周期延迟随机化。
- 当前 ROS/MuJoCo：腿 `Kd=0.8`；臂
  `Kp=[40,40,40,20,20,20]`、`Kd=[1,1,1,1,1,1]`；Gazebo URDF 没有
  joint damping/friction/armature 对应项。当前 ROS 臂力矩上限统一为 100 Nm，也与 MuJoCo
  的 `[20,20,15,7,5,5]` 不同。
- Gazebo 世界目前为 `max_step_size=0.002 s`、controller manager 500 Hz；训练与 MuJoCo
  均为 0.005 s / 200 Hz。第一阶段应做严格 0.005 s / 200 Hz 复现，之后再单独评估更小
  Gazebo 步长是否提高接触稳定性，不能同时改变多个变量。
- 训练地面摩擦默认 1.0，并随机化静/动摩擦到 0.5--1.2、恢复系数到 0--0.1；MuJoCo
  足端采用 `friction=0.8 0.02 0.01`。当前 Gazebo 没有显式统一机器人足端与地面的
  ODE 接触参数。

### 质量/惯量的权威来源与风险

- MuJoCo 与遗留 URDF 的小腿质量分别约 0.241352 kg 和 0.154 kg；Piper 若干 link 的
  质量/惯量差异更大（例如末端 link 约 0.457 kg 对 0.00563 kg）。
- 但训练 USD 是由遗留 URDF 转换而来，并启用了 `merge_fixed_joints=true`，所以不能把
  MuJoCo XML 直接当作训练动力学真值。训练复现阶段应以“训练 articulation 配置 + 实际
  训练 USD”为权威；只有在严格复现通过后，才统一到更真实的 CAD/URDF 惯量并进行随机化
  或再训练。
- 训练资产实际分层文件位于 `assets/go2_piper/configuration/`；robot 层可确认 articulation
  中包含 `FL/FR/RL/RR_foot` 和 `link0`。尝试用本机 Isaac Sim 4.5 备份中的 USD Python
  库读取 base 层时，底层资产因旧库与包含中文 prim/资源路径而报告重复/非法 spec，未能
  可靠导出惯量。
  后续实现需使用与生成该 USD 相同的 Isaac Sim/Usd 版本导出一份可审计参数表；当前分析以
  原始 URDF、转换配置和训练 articulation 配置三者交叉验证，不把失败读取结果当作依据。

### 稳定器的定位

- `base_stabilizer.cpp` 每个物理步直接覆写基座 z/roll/pitch 和速度，因此当前“完成任务”
  不能作为自然动力学稳定性的证据。它只能保留为显式 `demo_only` 诊断开关；sim2sim 验收
  路径必须默认关闭，且不能用虚拟抓取结果代替接触抓取结果。
- 训练侧显式使用 `preserve_order=True`，策略关节顺序就是 `FR, FL, RR, RL, joint1..6`；ROS `JOINT_NAMES` 与其一致。MuJoCo 部署中的重排只因为 MJCF 内部顺序为 `FL, FR, RL, RR`，所以 ROS 当前不需要再次重排。
- 策略观测的字段顺序、维度和缩放与 ROS 基本一致：角速度×0.2、投影重力、相对关节位置、关节速度×0.05、上次动作、3D 速度命令、7D 末端命令，三帧历史共 210 维。
- 训练执行器与 ROS 严重不一致：训练腿部 `Kp=30, Kd=0.6`，ROS 为 `30/0.8`；训练机械臂依次为 `Kp=[50,50,80,30,30,20]`、`Kd=[3,2,3,3,2.5,1]`，ROS 却为 `Kp=[40,40,40,20,20,20]`、`Kd=1`。
- 训练使用 `DelayedPDActuatorCfg`，含 `armature=0.01`、`friction=0.01` 和 0~4 个控制周期随机延迟；Gazebo URDF/ros2_control 尚未对齐这些执行器属性。
- 训练策略动作顺序与 ROS 一致，因此“再次交换左右/前后腿”会引入新错误，不应作为修复。

### 无稳定器修复前基线（2026-08-06）

- 已生成两份明确分离的 URDF：physics 模型只含 ros2_control/传感器插件；demo 模型额外含
  virtual grasp 和 base stabilizer。`wbc_gate.launch.py` 强制使用 physics 模型。
- 关闭稳定器后运行 10 秒门禁，控制器 ready 后立即出现显著横滚，首条报告为
  `roll=1.596 rad, pitch=-0.163 rad`，10 秒窗口累计 1633 个超限样本；门禁输出
  `WBC_GATE_FAIL`。这证明现有策略/模型在真实 Gazebo 动力学下不能站立，也证明新的
  physics 路径没有再被平面稳定器掩盖。
- Gazebo ros2_control 启动日志同时确认错误初始值确实被加载：FR/RR hip 为 `+0.1`，
  FL/RL hip 为 `-0.1`，与训练命名状态相反，而不是仅存在于 Python 常量中。

### 策略契约实施

- 新契约以训练侧显式关节名为索引，生成 URDF 后已确认初始髋角变为
  FR/RR `-0.1`、FL/RL `+0.1`。
- MuJoCo 不再依赖硬编码且注释错误的同一数组：由策略关节名和 MJCF 原始
  `FL,FR,RL,RR` 名称计算两个独立 permutation，默认角/增益/动作和观测均按方向映射。
- 契约还统一训练增益、动作裁剪 ±10、三帧历史、观测缩放和 200/50 Hz 比率；ROS 状态
  新增动作/力矩饱和率，后续可用于判断策略域外输出或执行器能力不足。

### 第一批修复后的 50 Hz PD 对照实验

- 修正髋角、训练增益、动作裁剪、足球/下段小腿以及 0.005 s 物理步后，在仍由 Python
  50 Hz 计算力矩的条件下，机器人在 WBC ready 前的 nominal settle 阶段已经倾覆。
- 门禁首个检查样本：`t=0.000 s, roll=1.850 rad, z=0.206 m`；10 秒共 999 个检查样本，
  996 个失败，最大 `|roll|=3.1296`、最大 `|pitch|=0.9765`、高度范围
  `0.1347--0.5051 m`。
- 这组结果不能说明模型修复无效，因为控制器激活后的力矩仍是 50 Hz 零阶保持；它证明
  200 Hz PD 内环和“控制器首个 update 即使用 nominal target”是进入下一轮实验的必要条件。

### 200 Hz ros2_control 内环结果

- 新增的 C++ effort controller 在 controller manager 每个 200 Hz update 中直接读取关节
  position/velocity，按共享契约计算并限幅 PD 力矩；50 Hz Python 节点只发布策略目标。
- 策略启用时，相比 50 Hz PD 明显改善：门禁首样本 roll 从 1.850 降到 0.004 rad，但
  pitch=0.285 rad 已略超 15°；随后仍倾覆，10 秒最大 roll=1.726、pitch=0.537 rad，
  高度最低 0.150 m。
- 关闭策略、仅运行 200 Hz nominal PD 的隔离实验 **通过**：1000/1000 样本合格，
  最大 `|roll|=0.0241`、最大 `|pitch|=0.0504`，高度范围 `0.2608--0.2698 m`。
- 因此现在可以排除“名义站姿本身无法承重”；剩余故障发生在策略目标激活后，下一步必须
  用实际 observation/action trace 与 MuJoCo 参考运行对比，不能继续盲调足端或 PD。

### Gazebo 策略 trace：IMU 反馈断链

- 3 秒策略 trace 共记录 324 个策略周期。所有记录中的 `imu_xyzw` 恒为 `[0,0,0,1]`，
  angular velocity 恒为 `[0,0,0]`；但同期 ground-truth odom 四元数从接近单位姿态变化到
  明显翻转。因此这不是噪声或坐标系小偏差，而是 `/imu/data` 根本没有送达 WBC。
- 生成 URDF 使用了插件不识别的 `<ros_topic_name>/imu/data</ros_topic_name>`；Humble
  `gazebo_ros_imu_sensor` 的输出端点应通过 `<ros><remapping>~/out:=imu/data</remapping>`
  配置。
- 失去 IMU 后策略观测中的三帧 projected gravity 永远为 `[0,0,-1]`，角速度永远为零，
  即使机器人翻转也无法闭环纠正。这足以解释“nominal PD 稳定、policy 一激活就倒”。
- 同一 trace 中机械臂 raw action 最大达到 10.85/13.25（裁剪前），估算关节速度最高超过
  100 rad/s；需要在 IMU 修复后复查。若仍异常，则继续比较末端命令分布和臂动力学。
- WBC 已增加 joint/IMU/odom freshness：任一传感器缺失或超过 0.1 s 时只发布 nominal
  target，且 ready=false，防止默认零数组被当作有效策略输入。

### 执行器动力学与命令契约修复后的门禁（2026-08-06）

- 纯 nominal trace 证明此前 Piper 在控制器激活瞬间已发生自由落体：修复前首帧
  `joint4=-2.47 rad`，最大速度约 `1456 rad/s`，虽然底盘门禁仍可能通过。
- 训练配置为全部执行器设置 `armature=0.01`、`friction=0.01`；Gazebo Classic 使用的
  SDFormat 1.7 没有 armature 字段。转换器现按关节轴把 `J*a*a^T` 加到 child-link inertia，
  作为等效广义转子惯量，并同步 Piper 力矩上限 `[20,20,15,7,5,5] Nm`。
- 修复后 nominal 10 秒门禁通过：0/999 失败，最大 roll/pitch 为
  `0.003293/0.017529 rad`；Piper 最大关节偏差降至 `0.077 rad`，joint4 最大速度降至
  `3.25 rad/s`。
- 指向目标姿态仍使策略门禁失败（638/1000 样本超限，最大 pitch `0.3585 rad`）；恢复
  训练 identity 姿态后，无稳定器、无状态覆写的策略 10 秒门禁通过：0/1001 失败，最大
  roll/pitch `0.067423/0.206188 rad`，高度 `0.272139--0.304726 m`。

### 动态门禁、D435 与 Piper 运动学对齐（2026-08-06）

- 42 秒命令扰动门禁通过：4201/4201 样本合格，最大 `|pitch|=0.259264 rad`、
  最大 `|roll|=0.071931 rad`、高度 `0.271640--0.309017 m`；机器人平面移动
  `0.303338 m`、偏航跨度 `0.629002 rad`，证明门禁不是静止输入。
- Gazebo physics 模式已实际收到 `/d435/camera/image_raw`、depth、points 和两路
  CameraInfo；此前“只能使用合成相机”的结论作废。合成源只留给 demo 模式。
- 同一组关节角下，旧 Piper URDF 的 TCP 为 `[0.1609,0.0008,0.0638]`，训练 MuJoCo
  模型为 `[0.4404,-0.0194,0.3316]`。转换器现同步训练模型 joint1--8 的 origin、axis、
  limit、link1--8 惯量，并增加训练定义的 `end_effector` 固定 frame；独立 FK 对比已逐位匹配。
- 清除残留 Gazebo master 后，对齐模型的干净策略门禁重新 spawn 成功并通过：1000/1000
  样本合格，最大 `|pitch|=0.224507 rad`、最大 `|roll|=0.042530 rad`、高度
  `0.274659--0.300650 m`。残留实例产生的早先“通过”结果已废弃。

### 20 关节 C++ 内环与物理抓取（2026-08-06）

- `PolicyPdController` 已扩展为独占 20 个 effort interface：18 维策略目标仍由
  `/go2_piper/policy_targets` 提供，双指目标独立来自 `/go2_piper/gripper/target`，从而删除
  Python 夹爪 effort 定时器和第二个控制器的接口竞争。
- 扩展到 20 个关节后曾发现 state interface 速度起始索引仍硬编码为 18，导致所有策略速度
  读取错位并使机器人失稳；改为按 `total_count=20` 取速度后，策略门禁再次通过：1000 个
  检查样本零失败，最大 pitch/roll 为 0.1593/0.0402 rad。
- 夹爪在控制器激活时从实测位置初始化，并限制目标变化率为 0.04 m/s；速度反馈改由 200 Hz
  位置差分获得，避免 Gazebo prismatic velocity 的接触尖峰。
- 双指 pad 摩擦提高到 10，闭合目标为 `[0.005,-0.005]`，在真实接触、无附着条件下已多次
  得到 0.058--0.070 m 的方块抬升，超过 0.03 m 验收门槛。
- 原 0.50 m 运输航点超出当前策略携物横移的可靠范围。最终场景采用两个互不重叠的
  0.18 x 0.30 m 工作台（中心 x=0.43/0.65 m），底盘目标 x=0.22 m，并新增底盘实际
  位移至少 0.20 m 的硬门槛；这仍是物理短程运输，而非原地放置。
- 动态 WBC 门禁原先在 31--36 秒把 TCP 目标设为 world z=0.42 m，低于工作台顶面
  z=0.506 m，并在约 35 秒重复触发 pitch 超限。该段实际测试的是夹爪撞桌恢复，不是
  自由空间稳定性；改为任务域内且无碰撞的 z=0.55/0.60 m，姿态阈值保持 15 度不变。

### 完成性复审：检测到抓取的数据流（2026-08-07）

- 当前 `physics_mission` 仍创建 `/gazebo/set_entity_state` 客户端，并在检测前、闭爪前各调用
  `_reset_cube()`；后一次会把方块放到实时 Link7/Link8 指垫中心。虽然闭爪后的抬升、运输、
  放置均为真实动力学，这仍不能证明检测结果驱动机器人自主接近目标。
- 现有非 `natural_grasp` 分支把“方块中心”直接当作 TCP 目标发布，忽略指垫中心相对 TCP
  的固定偏移，并且只等待固定时间，没有检测指垫对准误差，因此不能简单关闭 reset。
- WBC 节点已经具备 world PoseStamped 到训练 mixed-frame 命令的转换和 Piper 位置 IK；
  mission 可以用 D435 的 world 目标、当前 `end_effector` TF 和 Link7/Link8 指垫中心 TF
  计算误差，闭环更新 TCP 目标，无需 MoveIt 或 Gazebo 真值参与控制。
- 最终零覆写门禁应从 physics mission 源码和运行日志两方面证明：任务进程不导入/创建
  `SetEntityState`，启动后无 `/gazebo/set_entity_state` 调用，同时检测、抬升、运输、放置
  全链路仍通过。
- 首次取消状态覆写的物理运行在预抓取门禁正确失败：指垫误差从 0.143 m 未收敛，10 秒末仍为
  0.138 m。Gazebo 真值仅作审计时显示方块中心 x=0.503 m，而 D435 目标经旧补偿后为
  x=0.468 m，证明旧 `target_depth_offset=-0.015` 把可见前表面继续向相机方向偏移；50 mm
  方块应沿相机/机身前向增加约 0.025 m。
- 同一失败日志显示指垫存在约 0.12 m 的平面粗误差，而伺服循环仍向抓取前 station 发布回位
  速度，导致底盘与机械臂目标互相抵消。移动操作器闭环应采用底盘处理大尺度 XY 误差、机械臂
  保持世界 TCP 目标并完成 z/小尺度精调的分工。
- 第二轮底盘/机械臂同时闭环曾把误差降至 0.043 m，但连续横移后底盘失稳；第三轮改为足端
  世界锁定仍因目标相对基座达到约 0.58 m、超出可靠臂工作域而失稳。根因是任务启动后的
  8 秒 neutral arm settle 会先把 world 初始方块从 x=0.427 m 推到约 0.50 m。应在该等待前
  先用已验证的自由空间目标把机械臂抬到 x≈0.38 m、z=0.62 m，再检测尚未被碰撞的方块。
- 第四轮即使前置抬臂，轨迹仍从旧 spawn `(0.427,-0.065)` 穿过并把方块推到 x≈0.494，随后
  抬起的机械臂遮挡 D435 导致检测超时。任务 world 的初始方块应布置在取物台内且避开 neutral
  指垫扫掠区；选择 `(0.38,0.05,0.531)`，其完整底面仍在取物台上，并与初始指垫横向错开。

## Technical Decisions（技术决策）
| 决策 | 理由 |
|----------|-----------|
| 新工作空间位于 `LeggedManip_Lab/ros2_ws` | 与现有项目同源，且不污染 `/home/lili/3d-navi` ROS1 工作空间 |
| 首先实现 Python TorchScript WBC 节点 + ros2_control effort position controller | 便于复用已有部署数学并快速验证 Gazebo sim-to-sim |
| WBC 站立和速度/末端跟踪是抓取前硬门槛 | Isaac/MuJoCo 到 Gazebo ODE 存在接触和执行器差异 |
| 适配器必须在 README 和插件日志中标为 demo-only | 避免把可重复的 ROS 集成演示误解为动力学验证 |

## Issues Encountered（遇到的问题）
| 问题 | 解决方案 |
|-------|------------|
| Gazebo 中策略支撑失败 | 保留策略关节输出，锁定演示底盘的 z/roll/pitch 并运动学积分 `/cmd_vel` |
| Gazebo RGB-D 图像缺失 | 根据 world 中已知方块和相机几何生成 RGB-D 测试源 |
| 无稳定双指接触 | 在 `VERIFY` 状态使用虚拟持物，释放后恢复方块动力学 |
| Gazebo GUI 全黑 | Compose 中禁用在线模型库、指定本地模型目录并挂载 `/dev/dri` |
| 机器狗外观残缺或完全不可见 | 构建时复制完整 DAE、恢复其 URI，并导出/配置 Gazebo 包模型路径 |
| GUI 中看似只有白色碎件 | 服务端模型正常；默认相机把整机置于左下且被终端遮挡。修正 world 相机后已截图验证完整显示 |
| 夹爪高增益下穿越/撞限位 | prismatic armature 必须增加平移质量而非转动惯量；夹爪反馈还需与 50 Hz 策略推理解耦 |

## Resources（资源）
- 工作空间说明：`ros2_ws/README.md`
- 策略权重：`mujoco/deploy/policy/go2_piper/wbc/policy.pt`
- 主启动：`go2_piper_bringup/launch/demo.launch.py`
