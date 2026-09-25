# 发现与决策

## Requirements（需求）
- 两份课程实验指导书：Docker 版、Ubuntu 22.04 + ROS 2 Humble 原生版。
- 包含依赖、电脑差异、编译、建图用途/功能/命令/运行/常见问题、四 launch 导航流程。
- 预留实验图片位置，后续由用户补充。
- 提供统一环境处理脚本。

## Research Findings（研究发现）
- 当前 Docker 服务使用 host 网络、ROS_DOMAIN_ID=42、X11、/dev/dri 和工作区挂载。
- Dockerfile 基于 go2-piper-humble-base:open3d；仓库没有该基础镜像定义。
- 原生运行关键固定路径为 /workspace/Go2Arm_sim2sim；WBC 策略位于 ros2_ws/policy/go2_piper/wbc/policy.pt。
- 建图入口 course1_mapping.launch.py 同时启动 Gazebo、WBC、MID-360、Fast-LIO、RViz 和键盘窗口。
- 地图保存服务为 /map_save（std_srvs/srv/Trigger），默认输出 maps/go2_course1_map.pcd。
- 转换命令 pcd_to_nav2_map 默认 0.05 m、Z=0.10..1.00、flood_fill，输出 PGM/YAML/preview。
- 四阶段导航入口：simulation_wbc、fastlio、open3d_loc、navigation。
- ROS 2 Humble Tier 1 对应 Ubuntu 22.04；项目使用 Gazebo Classic 11，仅建议 amd64，Ubuntu 24.04 不适合作为原生课程环境。
- Python 依赖必须保持 NumPy <2；Open3D 与 ROS Humble Python 3.10/系统 SciPy ABI 需要一致。

## Technical Decisions（技术决策）
| 决策 | 理由 |
|----------|-----------|
| Docker 版作为推荐路径 | 隔离 ROS、Torch、Open3D 和 NumPy ABI 差异 |
| 原生版限定 Ubuntu 22.04 amd64 | Gazebo Classic 11 在 Jammy 的 ROS/Ubuntu二进制支持最稳定 |
| 不让脚本自动安装 Docker | Docker 官方仓库/用户组配置涉及系统权限，默认自动修改风险高 |

## Issues Encountered（遇到的问题）
| 问题 | 解决方案 |
|-------|------------|
| NumPy 2 导致 Open3D/scipy 导入失败 | 使用 numpy<2，并避免 Conda 与系统 ROS Python 混用 |
| source 在宿主执行报不存在 | Docker 命令必须在容器内 source /opt/ros/humble/setup.bash |
| 多行命令被拆开 | 文档所有可复制启动命令保持单行 |

## Resources（资源）
- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html
- https://docs.docker.com/engine/install/ubuntu/
- https://classic.gazebosim.org/tutorials?cat=get_started&tut=install_ubuntu
