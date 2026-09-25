# 发现与决策

## Requirements（需求）
-

## Research Findings（研究发现）
-

## Technical Decisions（技术决策）
| 决策 | 理由 |
|----------|-----------|

## Issues Encountered（遇到的问题）
| 问题 | 解决方案 |
|-------|------------|

## Resources（资源）
-
# 调研发现

- 正确运行工程是 `/home/lili/Go2Arm_sim2sim`，其 compose 挂载为 `/workspace/Go2Arm_sim2sim`。
- `Go2Arm_sim2sim` 当前只有 `floor1_navigation.launch.py`、`floor1_navigation.world`、原有 Nav2 配置和 RViz；没有 `go2_config`、`open3d_loc`、`pcd_to_nav2_map`、`FAST_LIO_ROS2`、`livox_laser_simulation`。
- `LeggedManip_Lab` 具有上述导航包，以及 `course1_nav_2d.launch.py`、Fast-LIO/Open3D 配置和障碍物场景；这些需要迁移到 Go2Arm，而不是继续从 LeggedManip_Lab compose 启动。
- Go2Arm 容器实际启动检查已成功拉起 Gazebo、原版 robot_state_publisher、Fast-LIO、WBC 控制器和机械臂控制器。
- 启动检查发现：临时容器没有宿主机 `/tmp/go2_course1_map.pcd`；`twist_mux`/Nav2 lifecycle 缺 `libdiagnostic_updater.so`；`odom_relay` 重复声明 `use_sim_time`；镜像 NumPy 2.2.6 与 Go2Arm WBC 的 PyTorch ABI 不兼容。
- Go2Arm Dockerfile 原本已有 `ros-humble-diagnostic-updater`，但现有 `go2-piper-humble:latest` 未按该 Dockerfile 重建，运行时仍缺库；需要真正完成镜像重建或在容器内补装。

## 2026-08-26 拆分前成功版恢复

- 四个 launch 首次创建时间为 2026-08-26 01:32（北京时间），会话记录分界时间为 `2026-08-25T17:32:27Z`。
- `course1_nav_2d.launch.py` 当前修改时间为 2026-08-25 23:57，早于拆分，因此它是拆分前候选版；恢复重点是拆分后被继续修改的依赖源码和配置。
- 当前完整 launch 和四 launch 运行图已证明等价，但都出现 Open3D 错误匹配、Nav2 虚假到达和抓取检测超时，表明依赖状态已偏离成功版。
- JSONL 解析工具 `jq` 未安装，后续使用 Python 只读解析。
- 分界后补丁的净结果：Open3D C++ 和 YAML 的实验参数已在后续补丁中完全回退；`mission_server.py` 只残留新增的 `odom_frame_filter` 参数和回调过滤逻辑。
- 2026-08-26 00:09 的 `build/go2_piper_mission/.../mission_server.py` 是拆分前编译快照；与当前源码的唯一差异正是 `odom_frame_filter`。
- 当前四 launch 的节点、参数和默认值与拆分前 `course1_nav_2d.launch.py` 对应段一致；还需排除分界后通过非 `apply_patch` 命令修改的文件。
- 恢复后完整入口实测：`/map` 正常发布，PGM 成功读取为 240x166@0.05m，Nav2 map/navigation lifecycle 均 active；首次查询是 ROS daemon 发现缓存延迟，刷新后地图与所有 Nav2 话题完整出现。
- Fast-LIO `/Odometry` 和 `/cloud_registered`、Open3D PCD 加载、ICP 初始化、WBC 12 腿策略 + 2 夹爪关节均已启动。
- Nav2 早于 Fast-LIO TF 就绪时会暂时报 `camera_init` 与 `base` 未连通，随后自动连通并完成 lifecycle 激活；这是完整入口并行启动的原有短暂现象。
- 旧四入口实例在约 3 分钟无人工初始位姿后曾误匹配到 `(-22.5, 23.2, -4.0)`；该污染实例上 Nav2 会正确中止。这不是四 launch 缺节点，而是需要按原流程在定位正确后开始导航。
- 全新四入口实例按 `gazebo -> fastlio -> open3d_loc -> navigation` 启动，ICP fitness 0.696/0.698 被接受；Nav2 目标 `(0.5,0)` 约 4 秒到达 `(0.487,-0.027)`，`navigate_to_pose` 返回 `SUCCEEDED`，恢复次数 0。

## 2026-08-26 单入口精确恢复重新审计

- 之前的“恢复完成”只验证了节点启动和短距离 Nav2，没有验证桌边固定站位及真实抓取，不能作为成功结论。
- 本次必须以首次拆分时间 `2026-08-25T17:32:27Z` 为硬边界，恢复完整运行链，而不是只恢复 `course1_nav_2d.launch.py` 的三个参数。
