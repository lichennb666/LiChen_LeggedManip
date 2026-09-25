# 任务计划：[简要描述]

## Goal

将导航抓取完整运行链迁移到 `/home/lili/Go2Arm_sim2sim`，保持 Go2Arm 原有机械臂、WBC、抓取和场景入口，新增/接入 3D Fast-LIO、Open3D 重定位、PCD 转 2D 地图、Nav2 和 RViz 导航显示。
[用一句话描述最终状态]

## Next Step
[唯一下一步；阶段状态变化时同步更新]

## Current Phase

### Phase 1：确认运行工程与差异
**Status:** complete

已确认 `Go2Arm_sim2sim` 的 compose 使用 `/workspace/Go2Arm_sim2sim`，但缺少 `go2_config`、`open3d_loc`、`pcd_to_nav2_map`、`FAST_LIO_ROS2` 等导航包；`LeggedManip_Lab` 中的修改不能作为运行入口。

### Phase 2：迁移导航包、地图和启动文件
**Status:** complete

### Phase 3：Go2Arm 容器构建与启动检查
**Status:** in_progress

## Next Step

等待 diagnostic_updater apt 安装完成并重新保存 PCD 后，完成 Go2Arm 镜像最终启动检查。
Phase 1

## Phases

### Phase 1：需求与发现
- [ ] 理解用户意图
- [ ] 确认约束和验收条件
- [ ] 将发现写入 findings.md
- **Status:** in_progress

### Phase 2：规划与结构
- [ ] 确定方案
- [ ] 记录决策及理由
- **Status:** pending

### Phase 3：实现
- [ ] 按计划执行
- [ ] 增量验证
- **Status:** pending

### Phase 4：测试与验证
- [ ] 验证全部需求
- [ ] 将结果写入 progress.md
- **Status:** pending

### Phase 5：交付
- [ ] 复核输出
- [ ] 向用户交付
- **Status:** pending

## Decisions Made
| 决策 | 理由 |
|----------|-----------|

## Errors Encountered
| 错误 | 解决方案 |
|-------|------------|

## Recovery Phase 2026-08-26

### Phase 4：恢复拆分前已验证版
**Status:** in_progress

### Phase 5：按已验证版原样拆分四个 launch
**Status:** pending

### Phase 6：清理进程并验证完整入口和四入口
**Status:** pending

## Recovery Next Step

解析分界点后的全部源码补丁，逆向恢复到 2026-08-26 01:32 前状态。

## Recovery Completion 2026-08-26

### Phase 4：恢复拆分前已验证版
**Status:** complete

### Phase 5：按已验证版原样拆分四个 launch
**Status:** complete

### Phase 6：清理进程并验证完整入口和四入口
**Status:** complete

### Phase 7：精确恢复首次拆分前单入口并可视化验收
**Status:** in_progress

恢复边界：`2026-08-25T17:32:27Z`。审计并撤销边界后对 Go2Arm 运行链的全部写入，以 Gazebo、RViz、Open3D 正确初始化、桌边导航和真实抓取作为完成条件。

## Next Step

提取边界后所有 Go2Arm 写入命令与受影响文件，恢复其边界前内容。
