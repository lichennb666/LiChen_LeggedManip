# Go2 + Piper Sim2Sim 稳定性实施方案

## 1. 目标和边界

目标是让同一 TorchScript WBC 策略在 Isaac Lab、MuJoCo 和 ROS 2 Humble + Gazebo
Classic 中使用一致的观测、动作和执行器语义，并在 Gazebo 中不依赖基座锁定插件自然站立、
行走和控制机械臂。

下列机制不得进入 sim2sim 验收路径：

- `BaseStabilizer::SetWorldPose()` 对基座 z/roll/pitch 的锁定；
- 通过 `SetWorldPose()` 搬运方块的虚拟抓取；
- 为了“看起来稳定”而跳过真实关节反馈、接触或重力。

当前策略属于“学习策略输出关节位置目标 + 关节 PD”的 learned WBC，并不是传统 QP 型
全身控制器。因此稳定性首先取决于训练/部署契约和动力学一致性。

## 2. 已确认的根因（按优先级）

### P0：会直接导致控制失效

1. **ROS 默认站姿的髋关节符号反了。** 策略顺序为
   `FR, FL, RR, RL`，正确默认髋角应为 `[-0.1,+0.1,-0.1,+0.1]`；当前 ROS 为相反符号。
2. **内环 PD 只运行在 50 Hz。** 训练与 MuJoCo 是 200 Hz 物理/PD、50 Hz 策略；当前
   Gazebo 虽以 500 Hz 仿真和更新 controller manager，但力矩由 Python WBC 节点每 20 ms
   才计算一次并保持。
3. **足端碰撞体被转换脚本裁掉。** Gazebo 生成模型只有小腿上段圆柱，没有小腿下段和
   半径 0.022 m 的足球，触地点和支撑几何与训练/MuJoCo 不同。
4. **稳定器掩盖了自然动力学。** 当前插件每个物理步覆写基座位姿和速度，现有 demo 成功
   不能证明策略稳定。

### P1：显著缩小或改变策略工作域

1. 训练动作裁剪为 `[-10,10]`，ROS/MuJoCo 部署为 `[-20,20]`。
2. 训练腿 PD 为 `30/0.6`，机械臂 PD 为
   `[50,50,80,30,30,20]/[3,2,3,3,2.5,1]`；部署使用另一套增益。
3. Gazebo 关节缺少训练中的 `armature=0.01`、`friction=0.01` 对应建模，力矩上限也未统一。
4. Gazebo 为 0.002 s / 500 Hz，训练与 MuJoCo 为 0.005 s / 200 Hz；在建立基准前同时
   改变步长、增益和接触会让问题不可归因。
5. 训练末端命令相对 `link0`，姿态由目标方向自动生成；ROS 默认命令和任务命令没有统一
   复用该公式，且转换代码使用 base 位姿代替 `link0` TF。

### P2：动力学资产存在多套真值

- Isaac 训练 USD、遗留 URDF、MuJoCo XML 的质量、惯量、碰撞和关节限制并不完全一致；
  Piper 末端部分差异尤其大。
- 训练 USD 由 URDF 转换并启用 `merge_fixed_joints=true`。不能直接把 MuJoCo XML 的惯量
  覆盖到 Gazebo 后宣称与训练一致，必须先导出训练 USD 的刚体参数表。

## 3. 修改设计

### 阶段 A：建立唯一的策略契约和可比较日志

新增 `ros2_ws/src/go2_piper_wbc/config/policy_contract.yaml`，集中保存：

- 策略关节顺序、各后端原始顺序和双向 permutation；
- default joint positions、action scale、action clip；
- observation 字段、缩放、历史长度和四元数顺序；
- policy rate、inner-loop rate、PD 增益、力矩限制；
- 末端命令的 mixed-frame 定义。

修改位置：

- `go2_piper_wbc/policy_core.py`：加载并校验契约；提供带关节名的映射、命令生成和 trace
  序列化，不允许再复制裸数组。
- `go2_piper_wbc/wbc_node.py`：修正 ROS 默认髋角，动作裁剪为训练值，发布观测、动作、目标
  和饱和统计。
- `mujoco/.../go2_piper.py` 与 `config_wbc.yaml`：使用同一契约，明确 qpos 原始顺序和策略
  顺序，移除误导性注释与重复常量。
- `test_policy_contract.py`：用 18 个唯一哨兵值验证 ROS/MuJoCo 双向映射、default offset、
  210 维历史布局和 `wxyz/xyzw` 转换。

同时增加可重放 trace：每个策略周期记录时间戳、`q/dq`、IMU、7 维末端命令、210 维
观测、18 维动作、目标角和力矩。先比较重力释放前若干步，再比较接触后的统计量；接触后
不同物理引擎不要求逐点轨迹完全相同。

### 阶段 B：拆分 50 Hz 策略和 200 Hz PD 内环

推荐新增 `go2_piper_policy_controller`（ROS 2 `controller_interface` C++ 插件）：

- controller manager 以 200 Hz 调用 `update()`；
- 插件直接读取 18 个 position/velocity state interface，声明 18 个 effort command interface；
- WBC 推理节点仅以 50 Hz 发布目标关节角及序号；
- 内环每个 update 周期执行
  `tau = clamp(Kp*(q_target-q)-Kd*dq, limits)`；
- 目标消息超时后平滑回到安全站姿，不保持任意旧动作；
- 导出 update jitter、目标年龄、力矩饱和率和 deadline miss。

预计修改：

- 新增 `ros2_ws/src/go2_piper_policy_controller/`；
- `go2_piper_description/config/controllers.yaml` 改用新控制器，基准频率先设 200 Hz；
- `wbc_node.py` 删除 50 Hz 力矩计算，只保留策略推理和目标发布；
- `simulation.launch.py` 等待控制器 active、joint state/IMU/odom 均有效后才解除暂停。

若先做快速验证，可在 Python 节点临时拆成 50/200 Hz 两个 timer；它只用于确认根因，最终
以 C++ ros2_control 内环作为验收实现，避免 Python GIL 和 Torch 推理阻塞 PD deadline。

### 阶段 C：建立可用于接触控制的规范机器人模型

修改 `convert_legacy_urdf.py`，不能再简单裁掉足端：

- 将每条腿的小腿下段圆柱和足球作为 `*_calf` 的 collision 固定加入；
- 明确足端 ODE 摩擦、接触刚度/阻尼、`min_depth` 和 `max_vel`；
- 统一 `link0`（机械臂安装基准）命名或 TF 别名；
- 保留合法的质量、质心和惯量，禁止把 visual box 同时当作动力学真值；
- 加入 URDF 审计测试：4 个足球、18 个受控关节、无孤立刚体、正定惯量、有限 joint limit。

还需提供一个参数导出工具，将 Isaac USD、MuJoCo XML 和生成 URDF 按 link/joint 名输出 CSV
差异。训练复现优先使用训练 USD 参数；若决定改用更真实的 CAD/MuJoCo 参数，则应视为新的
动力学域，并在 Isaac 中重新随机化/微调策略。

涉及文件：

- `gazebo/go2_piper_description/urdf/go2_piper.urdf`（源资产清理或替换）；
- `ros2_ws/src/go2_piper_description/scripts/convert_legacy_urdf.py`；
- `ros2_ws/src/go2_piper_bringup/worlds/pick_transport.world`；
- `source/.../assets/go2_piper/go2_piper_articulation_cfg.py`（仅在确定新动力学域或重新训练时）。

### 阶段 D：命令、启动和安全状态机对齐

- 从 TF 获取 `world -> link0`，使用训练侧同一公式生成
  `[x_link0,y_link0,z_world,qw,qx,qy,qz_link0]`；
- 默认末端目标先限制在训练初始范围 `x=0.4--0.45,y=±0.05,z=0.5`，姿态自动指向目标；
- 将训练范围内的低速 `vx/vy/wz=±0.2` 作为首轮行走测试，不能直接用 curriculum 极值；
- 启动依次为：spawn（暂停）→ controllers active → 首帧传感器有效 → nominal PD settle 3 s
  仿真时间 → 填充 history → 开策略 → 解除任务门禁；
- ready 状态必须包含 `policy_active`、`inner_loop_alive`、传感器 freshness 和无 NaN，不能只
  检查 topic subscriber 数量。

修改 `wbc_node.py`、`simulation.launch.py`、`mission_server.py`、`wbc_gate.py`。

### 阶段 E：稳定性通过后再做真实抓取

- `base_stabilizer` 和 `grasp_plugin` 改成显式 `demo_only` launch 选项，physics 模式默认不加载；
- 修复两指夹爪 joint/collision/coupling，加入双侧接触和物体相对位姿验证；
- mission 的 attach 服务不得创建运动学搬运效果，只能读取/验证真实接触状态；
- 先通过站立和低速运动，再逐步加入机械臂 pose sweep、夹取、负载行走和放置。

## 4. 实施顺序与停止条件

1. **契约测试通过**：关节映射、默认角、观测、动作目标在三个后端一致。
2. **名义 PD 通过**：关闭策略，仅用正确默认姿态自然站立 10 s；失败则只查模型、PD 和接触。
3. **策略站立通过**：无稳定器、零速度命令 60 s；roll/pitch 峰值 < 15°，base z 在
   0.23--0.33 m，无 NaN，持续力矩饱和率 < 5%。
4. **训练域命令通过**：前后/侧向/转向各运行 20 s，无跌倒，速度跟踪 RMSE 和滑移率入日志。
5. **机械臂命令通过**：固定基座速度为零，在训练初始范围及 limit range 做分段 pose sweep，
   机身仍满足倾角/高度门限。
6. **鲁棒性通过**：至少 10 个种子，摩擦 0.5--1.2、质量/惯量 ±10%、控制延迟 0--4 步；
   成功率达到 90% 以上。
7. **真实任务通过**：无虚拟抓取完成检测、双指接触抓取、负载运输和放置，并验证方块最终
   位姿而不是只检查 action 返回成功。

任何阶段失败时只修改该阶段拥有的变量。例如名义 PD 都无法站立时，不应通过重训策略或
增加姿态奖励来掩盖模型错误。

## 5. 是否需要重新训练

第一轮不重新训练。先修复默认角、动作裁剪、关节映射、内环频率和足端碰撞；这些都是部署
错误，重训会掩盖根因。

仅当上述契约完全一致且无稳定器 Gazebo 仍无法通过鲁棒性门禁时，再修改训练侧：

- 扩大足端接触参数、机械臂质量/质心和附加载荷随机化；
- 将观测/执行延迟与 ROS 实测 jitter 纳入随机化；
- 增加 action-rate/torque saturation 约束和跌倒终止；
- 使用规范 URDF/USD 重新训练或小学习率微调，并重新导出 TorchScript 和契约版本号。

这样才能区分“部署实现错误”和“策略没有覆盖 Gazebo 动力学域”两类问题。
