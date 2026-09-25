# 任务计划：带把手箱体与双指固定关节辅助抓取

## Goal
保留 ROS 2 Humble、Piper 和现有 WBC，把默认演示收缩为无桌子、无视觉定位、无自动任务状态机的 Gazebo 手动抓取场景：键盘控制机器狗移动和夹爪开合，Link7/Link8 同时接触物体后创建 fixed joint，释放时销毁关节。

## Next Step
用户运行 `GUI=true ./docker/run_demo.sh` 进入最小手动抓取场景；用移动键和机械臂微调键对准把手，`G` 抓取、`F` 重试附着、`R` 释放。

## Current Phase
Phase 8

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

### Phase 6：需求回退与入口解耦
- [x] 明确删除桌面、颜色检测和自动任务状态机
- [x] 保留旧任务为非默认兼容入口
- **Status:** complete

### Phase 7：最小场景与手动控制
- [x] 新增仅含地面、机器人和抓取物的 world
- [x] 键盘加入底盘、机械臂、闭爪附着与释放张开
- [x] 默认脚本切换到手动抓取入口
- **Status:** complete

### Phase 8：动态验证与交付
- [x] 构建、静态检查和 SDF 验证
- [x] 验证默认入口未启动感知/mission
- [x] 验证真实双指接触可建立 fixed joint
- **Status:** complete

## Decisions Made
| 决策 | 理由 |
|----------|-----------|
| 箱体主体使用非绿色材质、把手使用绿色材质 | 保留原 HSV 检测器并让视觉目标直接落在把手而非箱体质心 |
| 把手作为箱体模型内独立 link，以 fixed joint 连接主体 | 双指只对把手接触，跨模型 fixed joint 挂到把手即可带动整个箱体 |
| 辅助抓取必须同时检测 Link7 与 Link8 接触，失败即拒绝附着 | 消除旧插件“无接触也可虚拟附着”的路径 |
| fixed joint 建立后不再逐帧 SetWorldPose，释放时销毁关节 | 对齐双指接触触发刚性约束的语义 |
| assisted_grasp 默认保留，virtualize_physical_transport 默认关闭 | 运输全程由固定关节承载，不再切换任务侧状态覆写 |
| 新默认 world 不含桌子与相机目标依赖 | 用户明确撤回桌面取放与视觉定位需求 |
| 键盘 G/R 分别执行闭爪附着与释放张开 | 将抓取和机器狗移动集中到单一 teleop 入口 |
| 旧 physics_mission 显式指定旧 world | 保留兼容能力，同时避免它污染新默认流程 |
| 通过 Gazebo ContactManager filter 订阅物理接触消息并缓存 0.25 s | ROS 服务线程不会再错过物理步中的短暂接触快照，同时仍要求真实双指接触 |

## Errors Encountered
| 错误 | 解决方案 |
|-------|------------|
| 全主目录关键词搜索进入 npm/微信缓存并产生超大输出 | 后续仅在 LeggedManip_Lab 源码、配置与模型目录内检索 |
| 沙箱无法访问 Docker daemon socket | 使用受控权限重跑既有 `docker/build_workspace.sh` |
| 首次全量构建残留已删除 `target_cube.sdf` 的 root-owned 生成软链接 | 在容器内只删除该失效 build 链接后，bringup 重建通过 |
| Humble 镜像未安装 pytest 命令 | 以 Python 直接加载并执行无 fixture 的测试函数 |
