# 进度日志

## Session: 2026-08-14

### Current Status
- **Phase:** 8 - 动态验证与交付（完成）
- **Started:** 2026-08-14

### Actions Taken（已执行操作）
- 恢复旧 ROS 2/WBC 抓取任务计划并确认新需求是其增量改造。
- 审计 world、目标 SDF、抓取插件、mission 参数与执行顺序。
- 确认旧 assisted grasp 是接触可选的逐帧位姿跟随，不是真正 fixed joint。
- 新增橙色箱体 + 绿色独立把手 SDF/world，保持 HSV RGB-D 检测算法不变。
- 抓取插件改为 Link7/Link8 同时接触 `target_box::handle` 后创建 `Link6` 到 handle 的 Gazebo fixed joint；detach 销毁关节；删除插件内所有逐帧位姿覆写、stabilize 和 assisted lift。
- mission 改为闭爪后才调用 attach，默认关闭 virtual transport，目标真值和抓取几何均使用箱体模型内把手偏移。
- 运行两轮默认无头任务，确认绿色把手检测进入 PREGRASP/DESCEND；针对箱体被接近扫动增加把手净空、质量和动态目标跟踪。
- 根据用户新反馈停止继续调视觉桌面任务，改造目标切换为最小手动抓取场景。
- 确认 `teleop_test` 不启动颜色检测和 mission，可作为新默认入口；旧 `physics_mission` 保留为显式兼容入口。
- 新增 `manual_grasp.world`：仅地面、光照、Go2+Piper 和落地带把手箱体，无桌子与视觉目标依赖。
- 键盘节点加入 I/K/J/L/U/O 末端 2 cm 微调、G 闭爪延迟附着、F 重试附着、R 释放张爪。
- 默认 `run_demo.sh` 改为进入手动 teleop；旧视觉 mission 只在 `LEGACY_MISSION=true` 时进入。
- 抓取插件改用 Gazebo ContactManager filter 持久接收接触消息，解决服务线程瞬时读取漏检。

### Test Results（测试结果）
| 测试 | 预期 | 实际 | 状态 |
|------|----------|--------|--------|
| Python 语法与 diff whitespace | 无语法/空白错误 | `py_compile`、`git diff --check` 通过 | 通过 |
| 箱体 SDF 与 world | Gazebo 11 可解析 | 两个 `gz sdf -k` 均 `Check complete` | 通过 |
| ROS 2 Humble 全量构建 | 8 个包构建成功 | `Summary: 8 packages finished` | 通过 |
| WBC/感知/任务纯函数回归 | 全部通过 | 27/27 通过 | 通过 |
| 插件运行时加载 | 参数指向箱体把手 | 日志确认 `Link7 + Link8 -> target_box::handle` | 通过 |
| 无接触防伪附着 | attach 必须失败 | `success=False, left=false, right=false` | 通过 |
| 默认动态任务 | 检测后进入接触抓取 | 绿色把手检测通过，但旧间隙模型被 WBC/IK 接近扫离台面；最终加固模型未重跑完整运输 | 未完成动态验收 |
| 最小 world SDF | Gazebo 11 可解析 | `gz sdf -k` 输出 `Check complete` | 通过 |
| ROS 2 Humble 最终全量构建 | 8 个包成功 | `Summary: 8 packages finished` | 通过 |
| 默认节点隔离 | 无 perception/mission | 节点清单无 color_detector、mission_server、start_mission | 通过 |
| 双接触 fixed joint | 真实闭爪后附着成功 | 隔离 WBC 漂移后服务返回 `success=True`、`fixed joint attached after Link7+Link8 handle contact` | 通过 |

### Errors（错误）
| 错误 | 解决方案 |
|-------|------------|
| `docker/build_workspace.sh` 无权访问 `/var/run/docker.sock` | 请求受控权限运行同一构建脚本 |
| bringup symlink_data 仍引用旧 `target_cube.sdf` | 容器内移除唯一失效 build 软链接后重建通过 |
| 容器内没有 `pytest` 可执行文件 | 直接加载并执行现有纯函数测试，不安装新依赖 |
