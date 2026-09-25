# 发现与决策

## Requirements（需求）
- 分析 `/home/lili/go2_course1_projects(1)` 是什么项目、具备哪些功能、如何跑通。
- 本轮先做分析与诊断；目标目录只读，不修改源码，不连接或驱动未明确授权的真机。

## Research Findings（研究发现）
- 项目外层只是解压目录，真正根目录为 `/home/lili/go2_course1_projects(1)/go2_course1_projects`，内部是 Ubuntu 22.04 + ROS 2 Humble 的教学型仿真项目。
- 项目包含三个课程实验：① Go2 激光/惯性建图、回环优化、定位导航与避障；② RGB-D/YOLO 目标识别与视觉跟随；③ Whisper 语音识别、意图解析、TTS、GUI 与任务执行。
- 共用 ROS 2 工作空间未附带 `build/install/log`，需要本机安装依赖后执行 colcon 构建。
- 核心包包括 Unitree Go2 + CHAMP Gazebo 模型与控制、FAST-LIO2、SC-PGO、Livox MID-360 仿真、Open3D 全局定位、PCD→Nav2 地图转换、视觉感知、语音命令和语音导航。
- 打包了 `yolo11l.onnx`，但 Whisper `medium` 模型未打包，首次使用默认会联网下载；在线视觉问答/意图解析需要用户自己的 `DASHSCOPE_API_KEY`。
- README 给出了三个高层入口，但项目 1 实际需要多个终端依次启动 Gazebo、FAST-LIO、SC-PGO 与 Nav2，必须进一步核对脚本和 launch 的精确组合。
- 环境脚本按自身位置动态计算 `COURSE1_ROOT`，并对路径全部加引号，所以当前目录名中的括号不会直接破坏脚本；但 README 的 `~/go2_course1_projects/...` 示例与实际路径不一致，照抄会失败，应先改名/建立无括号路径，或始终使用当前绝对路径。
- `setup_env.sh` 会主动清空其他工作空间的 AMENT/CMAKE/COLCON/PYTHONPATH，固定 source `/opt/ros/humble`，再加载本工作空间；这是为了避免此前项目环境污染，运行时必须在独立新终端使用。
- 依赖安装脚本会执行 sudo apt 和 pip 网络安装，属于有副作用操作，本轮分析不会直接运行；依赖检查要求 ROS 2/colcon/CMake、Gazebo Classic、Nav2、PCL/GTSAM，以及 numpy/OpenCV/ONNX Runtime/SciPy/sounddevice/faster-whisper/edge-tts。
- 项目 2 是纯 Gazebo RGB-D 黄色小车跟随闭环，状态机为 IDLE/FOLLOW/SEARCH；另带已打包 YOLO11 ONNX 通用识别与需要 DashScope Key 的视觉问答扩展。
- 项目 3 将文字/麦克风/音频文件输入经过 Whisper 或本地/在线意图解析生成 JSON 动作序列，再以开环时间控制 `/cmd_vel` 并用 TTS 播报；它是教学演示，不保证里程计闭环精度。
- 项目 1 推荐严格按 Gazebo → FAST-LIO → SC-PGO → Nav2 启动，使用 CHAMP 键盘探索、Scan Context 回环优化、点云转二维 `/scan`、Nav2 目标与避障，并通过 `/sc_pgo/save_map` 保存 PCD。
- 当前最接近的 `go2-piper-humble:latest` 镜像具备 Humble、Gazebo Classic、Nav2、gazebo_ros2_control、PCL 和 teleop，但仍缺 `pointcloud_to_laserscan`、`twist_mux`、GTSAM、ffmpeg/espeak-ng，以及 ONNX Runtime、sounddevice、faster-whisper、edge-tts、dashscope；不能未经补依赖就直接构建全部项目。
- colcon 能识别 21 个有效包，包括 9 个 CHAMP/teleop 包、Go2 描述与配置、Velodyne/Livox 仿真、FAST-LIO、SC-PGO、Open3D 定位、PCD 转图、视觉、语音和语音导航。
- rosdep 额外确认当前 Humble 镜像缺 `joint_state_publisher(_gui)`、`robot_localization`、`ecl_threads`、`twist_mux`、Open3D 和 GTSAM。项目自带 `install_dependencies.sh` 没有列出 Open3D、robot_localization、joint-state-publisher、ecl-threads，说明 README 的“一键安装”清单并不完整。
- `ament_python` 的 rosdep 报告来自三个 Python 包的标准 `<buildtool_depend>ament_python</buildtool_depend>` 未被该镜像 rosdep 数据解析；这不等于源码结构错误，构建时 Humble 已提供 ament Python 工具。

## Technical Decisions（技术决策）
| 决策 | 理由 |
|----------|-----------|

## Issues Encountered（遇到的问题）
| 问题 | 解决方案 |
|-------|------------|
| 在只读挂载根目录运行 `colcon list` 时，colcon 尝试创建 `log/` 导致只读文件系统错误 | 后续使用 `--log-base /tmp/...`，构建也使用独立的 `/tmp/build`、`/tmp/install`、`/tmp/log`，保持目标源码只读 |

## Resources（资源）
- 项目总览：`/home/lili/go2_course1_projects(1)/go2_course1_projects/README.md`
- 工作空间说明：`/home/lili/go2_course1_projects(1)/go2_course1_projects/ros2_ws/README.md`
