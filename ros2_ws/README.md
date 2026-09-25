# Go2 + Piper ROS 2 Humble 手动抓取 Demo

本工作空间在不改动原 ROS 1 工作空间的前提下，把仓库已有的 Go2-Piper
模型和 TorchScript 策略接入 ROS 2 Humble、`ros2_control` 与 Gazebo Classic，
并提供最小 Gazebo sim2sim 场景、键盘移动和双指接触辅助抓取。

## 快速运行

主机是 Ubuntu 20.04，因此 ROS 2 Humble 运行在 Ubuntu 22.04 容器中：

```bash
cd /home/lili/LeggedManip_Lab/ros2_ws
./docker/build_image.sh
./docker/build_workspace.sh
GUI=false ./docker/run_wbc_gate.sh
GUI=true  ./docker/run_demo.sh
```

默认场景只有地面、Go2+Piper 和一个预对齐的带把手物体，不启动相机、颜色检测或
自动任务状态机。启动完成后直接使用键盘：

- `W/S` 前进后退，`A/D` 左右横移，`Q/E` 转向，`X` 停止
- `I/K` 机械臂前后，`J/L` 左右，`U/O` 上下（每次 2 cm）
- `G` 闭合夹爪，并在 Link7、Link8 都接触把手后建立 fixed joint
- `F` 在保持闭爪时重试附着（若第一次按 `G` 时尚未形成双接触）
- `R` 解除 fixed joint 并张开夹爪，`Ctrl+C` 退出

插件不会因为按下 `G` 就隔空吸附：缺少任一手指接触时，终端会报告抓取失败。
抓住后继续使用移动键，箱体由 `Link6 ↔ target_box::handle` fixed joint 随机器人运输。

需要保留的旧视觉任务仅作为兼容入口，不再默认启动：

```bash
GUI=true LEGACY_MISSION=true ./docker/run_demo.sh
```

Compose 已关闭 Gazebo Classic 的在线模型库刷新，并使用镜像内本地模型目录；
否则在网络受限环境中会出现窗口已打开但 3D 区域全黑、`Sim Time=0` 的现象。

## 键盘遥操作与抓取

蓝色箱体与黄色把手直接内置在 `manual_grasp.world` 中并落在地面上；机器人以策略
自然位姿（臂中立位）spawn，不需要视觉定位。

`run_demo.sh` 与下面的专用脚本进入同一手动流程：

```bash
cd /home/lili/LeggedManip_Lab/ros2_ws
GUI=true ./docker/run_teleop_test.sh
```

脚本会先在后台启动 `teleop_test.launch.py`（Gazebo + 控制器 + WBC 策略），
等待 `/go2_piper/wbc/status` 报告 `ready=true` 后，前台进入键盘遥操作节点
（持续以 20 Hz 发布 `/cmd_vel`，松开按键约 0.15 s 后自动停止）。
无桌面环境使用 `GUI=false`。查看场景日志：`docker logs go2-piper-teleop-launch`。

`/cmd_vel` 指令现在经过 wbc 的指令适配层（限幅 ±1.0 m/s / ±1.0 rad/s、
加减速斜坡 1.0 m/s² / 2.0 rad/s²、死区 0.01、超时平滑停车），
便于后续直接接入 Nav2 等导航栈；参数在
`src/go2_piper_wbc/config/wbc.yaml` 中可调。

## 控制架构：这里的 WBC 是什么

仓库里的 “WBC” 不是经典的 QP/逆动力学全身控制器。它是训练得到的
TorchScript 全身策略：输入为三帧、每帧 70 维的历史观测（共 210 维），
输出 12 个 Go2 腿关节和 6 个 Piper 臂关节的位置目标；50 Hz Python 推理节点
将目标交给 200 Hz C++ `ros2_control` PD 内环。该内环还独立接收双指目标并统一
控制全部 20 个 effort interface。

主要接口：

- `/cmd_vel`：底盘速度命令
- `/go2_piper/wbc/ee_target`：末端目标点
- `/go2_piper/wbc/status`：策略控制器状态
- `/go2_piper/perception/detection`：稳定后的三维目标检测
- `/go2_piper/mission`：抓取运输 Action
- `/go2_piper/grasp/{attach,detach}`：双指接触固定关节的建立/释放服务

## 任务边界

默认流程保留策略站立和 WBC 控制，但不启动 D435 颜色/深度检测和 mission。
Link7/Link8 都接触 `target_box::handle` 时才允许建立固定关节，无接触或单指接触都会失败。
旧视觉任务和 `SetEntityState` 回归开关仅由 `LEGACY_MISSION=true` 显式进入。

这里的“WBC”仍是学习式全身策略加关节 PD，而不是 QP/逆动力学 WBC；当前验证属于
MuJoCo/Isaac 到 Gazebo ODE 的 sim2sim，不等价于真机安全认证。真机部署前仍需完成
碰撞保护、力/电流限制、状态估计、急停和实机域随机化验证。

## 包结构

- `go2_piper_description`：旧 URDF 到 ROS 2/Gazebo 模型的确定性转换
- `go2_piper_wbc`：策略推理、历史观测、PD 力矩与稳定性门控
- `go2_piper_perception`：RGB-D 输入和绿色把手检测器
- `go2_piper_mission`：抓取运输 Action 状态机
- `go2_piper_gazebo_plugins`：双指接触 fixed-joint 抓取与演示专用底盘适配器
- `go2_piper_bringup`：world、控制器参数和 launch 文件
- `go2_piper_interfaces`：消息与 Action 定义

运行前建议先执行 `run_wbc_gate.sh`；任务服务器只有在 WBC 状态就绪并收到有效
里程计后才接受执行。
