from pathlib import Path
from html import escape

OUT = Path('/tmp/course_docs/out')
OUT.mkdir(parents=True, exist_ok=True)

COMMON_CSS = r'''<!doctype html><html><head><meta charset="utf-8"><style>
@page { size: A4; margin: 20mm 18mm 18mm 18mm; }
body { font-family: "Noto Sans CJK SC", sans-serif; color:#1f2933; font-size:10.5pt; line-height:1.5; }
h1 { color:#174a70; font-size:18pt; border-bottom:2px solid #4f91bd; padding-bottom:5pt; page-break-before:always; }
h1.first { page-break-before:avoid; }
h2 { color:#24638b; font-size:14pt; margin-top:16pt; }
h3 { color:#315c73; font-size:11.5pt; margin-top:12pt; }
p { margin:4pt 0 7pt; } ul,ol { margin:4pt 0 8pt 20pt; } li { margin:2pt 0; }
code { font-family:"DejaVu Sans Mono",monospace; font-size:8.5pt; background:#edf2f5; padding:1pt 2pt; }
pre { font-family:"DejaVu Sans Mono",monospace; font-size:8pt; line-height:1.35; background:#16242d; color:#f4f7f8; padding:9pt; border-radius:4pt; white-space:pre-wrap; }
table { width:100%; border-collapse:collapse; margin:8pt 0 12pt; table-layout:fixed; }
th { background:#dceaf3; color:#174a70; font-weight:bold; } th,td { border:1px solid #9fb6c5; padding:5pt; vertical-align:top; }
.cover { text-align:center; padding-top:38mm; page-break-after:always; }
.cover .kicker { color:#4f91bd; letter-spacing:2pt; font-size:12pt; }
.cover h1 { page-break-before:avoid; border:0; font-size:28pt; margin:14mm 0 5mm; }
.cover .edition { font-size:17pt; color:#315c73; }
.cover .meta { margin-top:35mm; color:#526672; }
.callout { border-left:5px solid #e59b32; background:#fff5e4; padding:8pt 10pt; margin:9pt 0; }
.ok { border-left-color:#3f8f64; background:#edf8f1; }
.danger { border-left-color:#c94d4d; background:#fff0f0; }
.figure { height:62mm; border:1.5px dashed #7d9caf; background:#f5f8fa; color:#607987; display:flex; align-items:center; justify-content:center; text-align:center; margin:9pt 0 12pt; }
.small { font-size:8.5pt; color:#526672; } .pagebreak { page-break-before:always; }
</style></head><body>'''

def code(s): return f'<pre>{escape(s)}</pre>'
def fig(n, title): return f'<div class="figure">实验图片 {n}：{escape(title)}<br>（学生完成实验后在此处替换图片）</div>'

def cover(edition):
    return f'''<div class="cover"><div class="kicker">移动操作机器人课程实验</div>
<h1>Go2Arm 仿真建图、重定位、导航与抓取</h1><div class="edition">{edition}</div>
<div class="meta">适用项目：Go2Arm_sim2sim<br>ROS 2 Humble · Gazebo Classic · Fast-LIO · Open3D · Nav2<br><br>版本：2026-08-26</div></div>'''

def common_intro(mode):
    env = 'Docker 容器' if mode == 'docker' else 'Ubuntu 22.04 原生 ROS 2 Humble'
    return f'''<h1 class="first">1 实验概述</h1>
<h2>1.1 实验目标</h2><p>本实验在 <code>Go2Arm_sim2sim</code> 中完成 Go2 + Piper 机械臂的三维建图、点云地图转换、Open3D 全局重定位、Nav2 二维导航和桌前抓取。当前手册使用环境：<b>{env}</b>。</p>
<ol><li>理解 MID-360 仿真点云、IMU、TF 与 Fast-LIO 的数据链路。</li><li>保存 PCD，并转换为 Nav2 可读取的 PGM/YAML。</li><li>按职责启动仿真/WBC、Fast-LIO、Open3D 重定位和 Nav2。</li><li>使用 RViz 的 2D Pose Estimate 与 2D Goal Pose 完成定位和导航。</li><li>在固定桌前站位触发微调、抓取和抬升。</li></ol>
<h2>1.2 软件链路</h2><table><tr><th>阶段</th><th>输入</th><th>输出/作用</th></tr>
<tr><td>Gazebo + WBC</td><td>模型、世界、策略、控制命令</td><td>机器人动力学、关节控制、仿真传感器</td></tr>
<tr><td>Fast-LIO</td><td><code>/livox/lidar</code>、<code>/livox/imu</code></td><td>里程计、局部点云、三维 PCD</td></tr>
<tr><td>Open3D 重定位</td><td>实时子图、已有 PCD、初始位姿</td><td><code>map</code> 到里程计坐标系的定位结果</td></tr>
<tr><td>Nav2</td><td>PGM/YAML、TF、障碍观测、目标点</td><td>全局/局部路径和 <code>/cmd_vel</code></td></tr>
<tr><td>任务与抓取</td><td>桌前固定站位、视觉微调、按键触发</td><td>接近、夹取、抬升和恢复键盘控制</td></tr></table>
<div class="callout"><b>工程保护原则：</b>课程脚本只处理环境、依赖和编译，不改动已验证的导航、WBC、机械臂或抓取参数。</div>'''

def docker_env():
    return '''<h1>2 Docker 版环境准备</h1>
<h2>2.1 推荐条件</h2><table><tr><th>项目</th><th>要求</th><th>说明</th></tr>
<tr><td>主机系统</td><td>Ubuntu 20.04/22.04/24.04 x86_64</td><td>Ubuntu 22.04 最省事；24.04 建议仅使用容器版。</td></tr>
<tr><td>图形环境</td><td>X11 桌面、可访问 <code>/tmp/.X11-unix</code></td><td>Wayland 会话遇到 GUI 问题时切换 Ubuntu on Xorg。</td></tr>
<tr><td>硬件</td><td>建议 8 核 CPU、16 GB 内存、20 GB 空间</td><td>独显不是必需；仿真卡顿时降低 Gazebo 视图负载。</td></tr>
<tr><td>容器镜像</td><td><code>go2-piper-humble:latest</code></td><td>课程发布者应提供镜像 tar；Dockerfile 依赖未随仓库提供的基础镜像。</td></tr></table>
<h2>2.2 一键脚本</h2><p>在项目根目录执行。首次安装会请求 sudo 权限；重新登录后 Docker 用户组才生效。</p>'''+code('chmod +x scripts/setup_course_environment.sh\n./scripts/setup_course_environment.sh docker check\n./scripts/setup_course_environment.sh docker install\n./scripts/setup_course_environment.sh docker build')+'''
<p>如教师提供镜像文件，先加载：</p>'''+code('docker load -i go2-piper-humble.tar')+'''
<h2>2.3 手工检查</h2>'''+code("docker --version\ndocker compose version\ndocker image inspect go2-piper-humble:latest\nxhost +local:docker")+'''
<div class="callout danger"><b>不要在容器内部执行 docker 命令。</b>提示符为 <code>root@robot:/workspace/...</code> 时已经在容器内；<code>docker compose</code> 必须在主机终端执行。</div>
<h2>2.4 编译</h2>'''+code("docker compose -f ros2_ws/docker/compose.yaml run --rm go2-piper-humble bash -lc 'source /opt/ros/humble/setup.bash&&cd /workspace/Go2Arm_sim2sim/ros2_ws&&colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'")+fig(1,'Docker 环境检查及编译完成输出')

def native_env():
    return '''<h1>2 原生 ROS 2 Humble 环境准备</h1>
<h2>2.1 支持范围</h2><table><tr><th>项目</th><th>要求</th><th>原因</th></tr>
<tr><td>操作系统</td><td>Ubuntu 22.04 Jammy x86_64</td><td>ROS 2 Humble 的主要平台，且 Gazebo Classic 11 的课程依赖适配最稳定。</td></tr>
<tr><td>不建议</td><td>Ubuntu 20.04、24.04、ARM 原生</td><td>ROS 发行版、Gazebo Classic 或二进制 Python 包存在兼容差异；请使用 Docker 版。</td></tr>
<tr><td>Python</td><td>系统 Python 3.10</td><td>不要混用 Conda；Open3D 与 NumPy ABI 必须一致。</td></tr>
<tr><td>硬件</td><td>建议 8 核 CPU、16 GB 内存、20 GB 空间</td><td>Gazebo、Fast-LIO、Open3D 和 Nav2 会并行运行。</td></tr></table>
<h2>2.2 一键脚本</h2>'''+code('chmod +x scripts/setup_course_environment.sh\n./scripts/setup_course_environment.sh humble check\n./scripts/setup_course_environment.sh humble install\n./scripts/setup_course_environment.sh humble build')+'''
<p><code>install</code> 会安装 ROS 2 Humble、Gazebo Classic、Nav2、ros2_control、PCL/Open3D 相关依赖，固定 <code>numpy&lt;2</code>、<code>open3d==0.18.0</code>、CPU 版 PyTorch，并创建兼容工程硬编码路径的符号链接：</p>'''+code('/workspace/Go2Arm_sim2sim -> 当前项目根目录')+'''
<h2>2.3 编译</h2>'''+code('cd ~/Go2Arm_sim2sim/ros2_ws\nsource /opt/ros/humble/setup.bash\nrosdep install --from-paths src --ignore-src -r -y\ncolcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release\nsource install/setup.bash')+'''
<div class="callout danger"><b>Python ABI：</b>若出现 “compiled using NumPy 1.x cannot be run in NumPy 2.x”，执行 <code>python3 -m pip install --user --force-reinstall "numpy&lt;2" "open3d==0.18.0"</code>，关闭旧终端后重试。</div>'''+fig(1,'原生 Humble 环境检查及编译完成输出')

def mapping(mode):
    prefix = "docker compose -f ros2_ws/docker/compose.yaml run --rm go2-piper-humble bash -lc 'source /opt/ros/humble/setup.bash&&source /workspace/Go2Arm_sim2sim/ros2_ws/install/setup.bash&&ros2 launch go2_piper_bringup course1_mapping.launch.py'" if mode=='docker' else 'ros2 launch go2_piper_bringup course1_mapping.launch.py'
    enter = "另开主机终端，先用 `docker ps` 找到运行中的容器，再执行 `docker exec -it 容器名 bash`，并 source ROS 环境。" if mode=='docker' else "另开终端并执行 `source /opt/ros/humble/setup.bash && source ~/Go2Arm_sim2sim/ros2_ws/install/setup.bash`。"
    save_cmd = code('ros2 service call /map_save std_srvs/srv/Trigger "{}"')
    convert_cmd = code('cd /workspace/Go2Arm_sim2sim/ros2_ws\nsource /opt/ros/humble/setup.bash\nsource install/setup.bash\nros2 run pcd_to_nav2_map convert --input ../maps/go2_course1_map.pcd --output ../maps/go2_course1_map --resolution 0.05 --force')
    return f'''<h1>3 三维建图实验</h1>
<h2>3.1 程序用途与功能</h2><p><code>course1_mapping.launch.py</code> 一次启动 Gazebo 场景、Go2Arm 模型、WBC/控制器、MID-360 仿真驱动、Fast-LIO、三维 RViz 和按住移动/松开停止的键盘窗口。默认地图保存到 <code>/workspace/Go2Arm_sim2sim/maps/go2_course1_map.pcd</code>。</p>
<h2>3.2 启动建图</h2>{code(prefix)}
<p>建议等待 Fast-LIO 输出 <code>IMU Initial Done</code> 和 <code>Initialize the map kdtree</code> 后再移动。使用建图键盘窗口：W/S 前后、A/D 横移、Q/E 转向、X 或空格停止；线速度约 0.20 m/s，转向约 0.40 rad/s。</p>
{fig(2,'Gazebo 建图场景与键盘窗口')}{fig(3,'RViz 中水平且连续的 Fast-LIO 三维点云')}
<h2>3.3 保存 PCD</h2><p>{enter}</p>{save_cmd}
<p>返回 <code>success=True, message='Map saved.'</code> 后可 Ctrl+C 结束建图。先结束程序再调用服务会导致服务不存在。</p>
<h2>3.4 PCD 转 PGM/YAML</h2>{convert_cmd}
<p>输出包括 <code>go2_course1_map.pgm</code>、<code>go2_course1_map.yaml</code> 和 <code>go2_course1_map_preview.png</code>。默认按高度区间提取障碍，并使用 flood-fill 标记可通行区域。</p>{fig(4,'PCD 转换后的二维地图预览')}
<h2>3.5 建图验收</h2><ul><li>Gazebo 中仅有一套雷达外观，点云与机器人运动一致。</li><li>静止时墙面点云稳定，不持续漂移或倾斜。</li><li>PCD 文件非空，PGM 中外墙、桌子和障碍物轮廓可辨识。</li><li>YAML 中 image 路径与 PGM 文件对应。</li></ul>'''

def launches(mode):
    if mode == 'docker':
        cmds = [f"docker compose -f ros2_ws/docker/compose.yaml run --rm go2-piper-humble bash -lc 'source /opt/ros/humble/setup.bash&&source /workspace/Go2Arm_sim2sim/ros2_ws/install/setup.bash&&ros2 launch go2_piper_bringup {n}'" for n in ['simulation_wbc.launch.py','fastlio.launch.py','open3d_loc.launch.py','navigation.launch.py']]
    else:
        cmds = [f'ros2 launch go2_piper_bringup {n}' for n in ['simulation_wbc.launch.py','fastlio.launch.py','open3d_loc.launch.py','navigation.launch.py']]
    rows = [('终端 1','simulation_wbc.launch.py','Gazebo、机器人、MID-360、WBC、关节控制器','控制器均 Configured and activated'),('终端 2','fastlio.launch.py','订阅 LiDAR/IMU，发布 Fast-LIO 里程计与点云','IMU Initial Done；map kdtree 初始化'),('终端 3','open3d_loc.launch.py','加载 PCD，执行 Open3D ICP 全局重定位','ICP 日志 accepted'),('终端 4','navigation.launch.py','加载 PGM/YAML、启动 Nav2、RViz、任务与键盘','地图已加载；Nav2 节点 active')]
    table=''.join(f'<tr><td>{a}</td><td><code>{b}</code></td><td>{c}</td><td>{d}</td></tr>' for a,b,c,d in rows)
    blocks=''.join(f'<h3>终端 {i}</h3>{code(c)}' for i,c in enumerate(cmds,1))
    return f'''<h1>4 四阶段导航启动</h1><p>四个 launch 是单 launch 工作流的职责拆分，必须按顺序启动。每个进程保持运行，达到等待条件后再开启下一个终端。</p>
<table><tr><th>顺序</th><th>Launch</th><th>职责</th><th>进入下一步的条件</th></tr>{table}</table>{blocks}
<div class="callout"><b>Docker 说明：</b>四条命令使用四个主机终端，但属于同一个 Compose 工程和 host network，可通过 ROS_DOMAIN_ID 42 互相发现。不要把命令拆成换行后逐段执行。</div>
<h2>4.1 RViz 操作</h2><ol><li>Global Options 的 Fixed Frame 设为 <code>map</code>。</li><li>Map 显示项 Topic 设为 <code>/map</code>。</li><li>点击工具栏 <b>2D Pose Estimate</b>，在机器人真实起点附近拖出朝向。</li><li>等待 Open3D ICP 连续 accepted，点云、机器人和地图不再跳动。</li><li>点击 <b>2D Goal Pose</b> 给出目标。任务逻辑会把桌旁附近目标映射到已验证的固定预抓取站位。</li></ol>
{fig(5,'Open3D 实时子图与 PCD 对齐效果')}{fig(6,'二维地图、全局路径、局部路径和机器人位姿')}
<h2>4.2 桌前抓取</h2><p>导航到桌前固定站位后，在交互终端按 <b>P</b> 启动微调和后续抓取。微调、夹取和抬升期间任务节点应独占速度/机械臂控制，避免键盘命令干扰；抬升完成后恢复底盘键盘控制。</p>
<p>当前固定抓取映射目标为约 <code>map=(7.87, 4.30, yaw=-0.247)</code>，区域中心约为 <code>(7.55, 4.30)</code>。这些是课程工程参数，不要仅凭 RViz 外观随意修改。</p>
{fig(7,'机器人到达桌前固定站位')}{fig(8,'夹取并抬升目标物成功')}
<h2>4.3 实验结束</h2><p>按终端 4 → 3 → 2 → 1 的逆序 Ctrl+C。若 Gazebo 或 RViz 残留，先确认没有其他实验正在运行，再清理对应进程或容器。</p>'''

def troubleshooting(mode):
    docker_rows = '<tr><td><code>service go2-piper-humble is not running</code></td><td><code>exec</code> 只适用于长驻容器；当前流程使用 <code>run --rm</code></td><td>直接按本手册四条 <code>run --rm</code> 命令启动；需要进入时用 <code>docker ps</code> + <code>docker exec</code>。</td></tr><tr><td>容器中 <code>docker: command not found</code></td><td>在容器内部误执行 Docker CLI</td><td>退出到 <code>lili@robot:~/Go2Arm_sim2sim$</code> 主机提示符。</td></tr>' if mode=='docker' else ''
    diagnose_cmd = code('ros2 topic list | sort\nros2 node list\nros2 service list | grep map\nros2 topic hz /livox/lidar\nros2 topic hz /livox/imu\nros2 run tf2_ros tf2_echo map base_link\nros2 control list_controllers')
    return f'''<h1>5 常见问题与排查</h1><table><tr><th>现象</th><th>主要原因</th><th>处理</th></tr>
{docker_rows}
<tr><td>第二行提示 <code>command not found</code></td><td>长命令复制后换行，续行符缺失</td><td>使用本手册单行命令；选项必须写成 <code>--free-space-mode</code>，不能少一个短横线。</td></tr>
<tr><td><code>Package ... not found</code></td><td>未 source 工作空间，或未编译</td><td><code>source /opt/ros/humble/setup.bash</code> 后再 <code>source ros2_ws/install/setup.bash</code>。</td></tr>
<tr><td>Open3D 未安装/NumPy ABI 报错</td><td>Open3D 与 NumPy 2.x 或 SciPy 二进制不兼容</td><td>固定 <code>numpy&lt;2</code> 与 <code>open3d==0.18.0</code>，不要混用 Conda。</td></tr>
<tr><td>PCD input file does not exist</td><td>当前目录是 <code>ros2_ws</code>，地图实际在项目根目录 <code>maps</code></td><td>输入使用 <code>../maps/go2_course1_map.pcd</code>。</td></tr>
<tr><td><code>/map_save</code> 一直等待</td><td>建图 launch 未运行、已 Ctrl+C，或服务尚未启动</td><td>保持建图运行，等待出现点云后，在同一 ROS_DOMAIN_ID 的终端调用。</td></tr>
<tr><td>RViz 空白或点云晚出现</td><td>IMU 初始化和 KD-tree 构建需要时间</td><td>等待初始化日志；检查 <code>/livox/lidar</code>、<code>/livox/imu</code> 和 Fixed Frame。</td></tr>
<tr><td>点云倾斜、反向或漂移</td><td>雷达安装 TF、IMU 外参或重复 TF/里程计发布不一致</td><td>保持建图和导航使用同一模型与 Fast-LIO 配置；不要在 RViz 中人为旋转数据；检查是否启动了重复传感器/TF 发布者。</td></tr>
<tr><td>Gazebo 报 <code>Missing element description for [ray]</code> 并退出</td><td>SDF 传感器标签错误或 install 中仍是旧文件</td><td>修正源文件后重编相关包并重新 source；Gazebo 退出会连带导致 controller_manager 不可用。</td></tr>
<tr><td>RViz Initial Pose 插件类不存在</td><td>把工具消息类型误配置成 Display 插件</td><td>删除错误的 Initial Pose Display；使用顶部工具栏的 2D Pose Estimate。</td></tr>
<tr><td>Map 显示 No map received</td><td>地图服务未加载、Topic/Fixed Frame 错误</td><td>确认终端 4 已加载 YAML；Map Topic=<code>/map</code>，Fixed Frame=<code>map</code>。</td></tr>
<tr><td>2D Goal 后不规划</td><td>尚未稳定定位、Nav2 未 active、目标在地图外或不可达</td><td>先完成 2D Pose Estimate 并确认 ICP accepted，再检查全局/局部 costmap 状态。</td></tr>
<tr><td>到桌前但不抓取</td><td>只完成导航，未按 P；任务状态或控制权未切换</td><td>确认进入固定站位、终端无旧任务残留，然后按 P；检查任务节点、视觉结果和机械臂控制器日志。</td></tr></table>
<h2>5.1 快速诊断命令</h2>{diagnose_cmd}
<div class="callout danger"><b>清理进程前先确认：</b>同一台电脑可能有其他 ROS/Gazebo 实验。不要用无差别的 kill 命令误关他人程序。</div>'''

def appendix(mode):
    return '''<h1>6 实验记录与验收</h1>
<h2>6.1 学生记录</h2><table><tr><th>项目</th><th>填写内容</th></tr><tr><td>电脑/系统</td><td>CPU、内存、显卡、Ubuntu 版本、Docker 或原生</td></tr><tr><td>软件版本</td><td>ROS 2、Gazebo、Open3D、NumPy、Docker/Compose</td></tr><tr><td>建图结果</td><td>PCD 大小、PGM 分辨率、占据/空闲/未知比例</td></tr><tr><td>定位结果</td><td>初始位姿、ICP 是否连续 accepted、是否漂移</td></tr><tr><td>导航结果</td><td>目标、规划是否成功、到达误差、耗时</td></tr><tr><td>抓取结果</td><td>尝试次数、成功次数、失败阶段及日志</td></tr></table>
<h2>6.2 验收清单</h2><ul><li>环境检查无阻断项，13 个工作空间包可编译。</li><li>可控制机器人完成场景覆盖，保存 PCD 并转换 PGM/YAML。</li><li>四个 launch 按顺序启动，各阶段达到等待条件。</li><li>二维地图、机器人、点云和路径在 RViz 中坐标一致。</li><li>2D Goal Pose 能产生路径并到达目标。</li><li>桌前按 P 后完成微调、夹取和抬升。</li></ul>
<h2>6.3 思考题</h2><ol><li>为什么建图需要三维点云，而 Nav2 使用二维 PGM？</li><li>雷达 TF 的俯仰误差为何会表现为倾斜墙面和定位漂移？</li><li>为什么必须等待 Fast-LIO 初始化后再启动 Open3D 重定位？</li><li>导航速度与 WBC 稳定性、定位质量之间有什么关系？</li><li>抓取阶段为什么需要临时屏蔽底盘和机械臂的手动控制？</li></ol>
<h2>6.4 官方参考</h2><ul><li>ROS 2 Humble Release: https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html</li><li>ROS 2 Humble Ubuntu 安装: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html</li><li>Docker Engine Ubuntu 安装: https://docs.docker.com/engine/install/ubuntu/</li><li>Gazebo Classic Ubuntu 安装: https://classic.gazebosim.org/tutorials?cat=get_started&amp;tut=install_ubuntu</li><li>Gazebo Classic EOL: https://community.gazebosim.org/t/gazebo-classic-end-of-life/2563</li></ul>
<p class="small">说明：Gazebo Classic 已结束生命周期，本课程为保持既有模型与插件兼容继续使用 Gazebo 11。新课程环境建议封装为已验证镜像，并记录镜像摘要。</p>'''

def build(mode):
    edition = 'Docker 容器版实验指导书' if mode=='docker' else 'Ubuntu 22.04 + ROS 2 Humble 原生版实验指导书'
    env = docker_env() if mode=='docker' else native_env()
    body = cover(edition)+common_intro(mode)+env+mapping(mode)+launches(mode)+troubleshooting(mode)+appendix(mode)
    html = COMMON_CSS+body+'</body></html>'
    stem = 'Go2Arm课程实验指导书_Docker版' if mode=='docker' else 'Go2Arm课程实验指导书_ROS2_Humble原生版'
    (OUT/f'{stem}.html').write_text(html,encoding='utf-8')
    # Editable text source: HTML is intentionally retained; Markdown companion points students to the DOCX workflow.
    md = f'# {edition}\n\n本文件是课程材料的轻量索引。完整正文、表格和图片占位框见同名 DOCX。\n\n## 核心命令\n\n- 环境检查：`./scripts/setup_course_environment.sh {mode} check`\n- 安装依赖：`./scripts/setup_course_environment.sh {mode} install`\n- 编译工程：`./scripts/setup_course_environment.sh {mode} build`\n- 建图：`ros2 launch go2_piper_bringup course1_mapping.launch.py`\n- 保存：`ros2 service call /map_save std_srvs/srv/Trigger "{{}}"`\n- 四阶段：`simulation_wbc.launch.py` → `fastlio.launch.py` → `open3d_loc.launch.py` → `navigation.launch.py`\n\n> 修改完整指导书内容时，可编辑同名 HTML 后用 LibreOffice 导出 DOCX。\n'
    (OUT/f'{stem}.md').write_text(md,encoding='utf-8')

build('docker')
build('humble')
