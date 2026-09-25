# 任务计划：ROS 2 Humble Go2-Piper RL-WBC 抓取运输

## Goal
在 Ubuntu 22.04 Docker 中交付可构建、可测试的 ROS 2 Humble + Gazebo Classic Go2-Piper 工程，以现有 TorchScript RL-WBC 联合控制 12 腿关节和 6 臂关节，完成绿色方块 RGB-D 检测、物理抓取、固定航点运输和投放；进一步消除平面稳定与虚拟抓取适配器，完成可量化验证的 Isaac/MuJoCo→Gazebo sim2sim。

## Next Step
审计 physics mission 中所有 Gazebo `SetEntityState` 调用，随后实现 D435 目标到双指指垫中心的闭环视觉伺服，取消闭爪前方块重置。

## Current Phase
Phase 9

## Phases

### Phase 1：需求与发现
- [x] 理解用户意图
- [x] 确认约束和验收条件
- [x] 将发现写入 findings.md
- **Status:** complete

### Phase 2：规划与结构
- [x] 确定方案
- [x] 记录决策及理由
- **Status:** complete

### Phase 3：实现
- [x] 按计划执行
- [x] 增量验证
- **Status:** complete

### Phase 4：测试与验证
- [x] 验证全部需求
- [x] 将结果写入 progress.md
- **Status:** complete

### Phase 5：交付
- [x] 复核输出
- [x] 向用户交付
- **Status:** complete

### Phase 6：Gazebo GUI 黑屏修复
- [x] 在用户 X11 会话中复现黑屏
- [x] 检查 gzclient、Gazebo master、OGRE 与模型库日志
- [x] 禁用在线模型库等待并映射本机渲染设备
- [x] 截图确认场景显示、仿真时间推进和任务完成
- **Status:** complete

### Phase 7：Go2/Piper 外观模型恢复
- [x] 根据用户截图确认机器人只显示零碎 mesh
- [x] 对比旧 URDF、转换后 URDF 和 MuJoCo 拆分资源
- [x] 恢复 Go2/Piper 完整 DAE 视觉资源
- [x] 重新构建并截图验证完整机器狗与机械臂
- **Status:** complete

### Phase 8：Sim2Sim 控制稳定性分析
- [x] 审计训练域、MuJoCo 部署与 Gazebo 的观测/动作契约
- [x] 对比动力学、执行器、接触和控制时序参数
- [x] 采集无稳定器模式下的定量失稳证据
- [x] 修复部署契约、200 Hz 内环、IMU、执行器 armature/力矩上限与命令姿态
- [x] 无稳定器策略闭环 10 秒站立门禁通过
- [x] 完成长时站立与速度/末端扰动门禁
- [x] 完成真实 RGB-D、双指接触和物理抓取运输门禁
- [x] 输出修改文件、实施顺序和验收指标
- **Status:** complete

### Phase 9：检测驱动自主接近与零状态覆写抓取
- [ ] 审计并禁止 physics mission 在任务期间写入方块/机器人状态
- [ ] 用 D435 检测坐标和 Link7/Link8 指垫 TF 闭环对准目标
- [ ] 以视觉伺服误差门禁替代一次性方块重置
- [ ] 完成无状态覆写的检测、物理抓取、抬升、运输和独立台面放置
- [ ] 重跑 10 秒/42 秒 WBC、pytest、全量构建与 GUI 默认入口验证
- **Status:** in_progress

## Decisions Made
| 决策 | 理由 |
|----------|-----------|
| ROS 2 Humble 运行于 Ubuntu 22.04 Docker | 宿主是 Ubuntu 20.04 且无 Humble，隔离现有 Noetic 环境 |
| 使用 Gazebo Classic + gazebo_ros2_control | 用户明确选择该仿真器 |
| 使用现有 210D 输入/18D 输出 RL-WBC 策略 | 本机已有权重，且满足腿臂联合控制要求 |
| 不使用 MoveIt 2 | 抓取末端轨迹由 WBC 位姿指令实现，避免控制器争用 |
| 旧 URDF 构建时派生为 ROS 2 URDF | 保留用户现有 ROS1 文件和未提交修改 |
| 演示专用稳定器只保留在 demo URDF | physics URDF 已在无状态覆写条件下通过策略动态门禁 |
| physics_mission 使用 Gazebo D435 原始 RGB/depth | 实测插件可稳定发布图像、深度、点云和 CameraInfo |
| physics_mission 禁止虚拟附着，并查询 Gazebo 实体状态验收 | 成功必须证明方块被实际抬升、运输和放到目标台 |
| sim2sim 验收默认禁用平面稳定器和虚拟抓取 | 两者直接覆写物理状态，只能用于 ROS 任务链演示，不能证明控制稳定或物理抓取 |
| 先修部署契约再考虑重训 | 默认角、动作裁剪、PD 频率和足端碰撞均存在明确实现错误，重训会掩盖根因 |
| 最终拆分 50 Hz 策略与 200 Hz C++ ros2_control PD 内环 | 与训练时序一致，并避免 Python/Torch 阻塞力矩更新 deadline |
| 不把闭爪前 `SetEntityState` 对齐视为自主抓取完成 | 方块由检测结果驱动机械臂/底盘接近才符合检测抓取任务；episode reset 只能发生在任务启动前的 world 初始化 |

## Errors Encountered
| 错误 | 解决方案 |
|-------|------------|
| 宿主 Python 3.8 无 `xml.etree.ElementTree.indent` | 仅在 API 存在时格式化，保持转换器兼容 Focal/Jammy |
| Docker Hub 拉取 `humble-desktop-full-jammy` 元数据超时 | 发现本机已有同内容 `osrf/ros:humble-desktop-full`，改为离线复用该镜像 |
| 直接 Gazebo 动力学下机身倾覆、平移发散 | 先证明真实门控会失败，再增加显式 demo-only 平面稳定器并重新通过门控 |
| D435 Gazebo 插件不发布 RGB/depth 图像 | 增加确定性合成 RGB-D 源，检测器仍从图像计算三维目标 |
| 腕部与方块距离随策略漂移至 3.52 m | 删除误导性的距离判定，改成明确命名和记录的虚拟抓取适配器 |
| Gazebo GUI 窗口存在但视图区全黑、Sim Time 为 0 | gzclient 卡在在线模型库刷新；设置空 `GAZEBO_MODEL_DATABASE_URI` 并指定本地模型路径 |
| 机器人只显示一个零碎白色部件 | 转换器误用拆分 STL 的 `*_0` 单片；恢复完整 Go2/Piper DAE，并将 ROS 安装空间加入 Gazebo 模型搜索路径 |
| Isaac Sim 4.5 备份 USD 库无法读取训练 base USD | 旧 USD 库将中文 prim 解码为非法/重复 spec；记录为资产审计工具链问题，暂以 articulation 配置、USD robot 层和源 URDF 交叉核对 |
