# Course1 Sim2Sim: Mapping + 2D导航 + 桌边抓取（GO2Arm）

## 1) 建图阶段

1. 启动建图：
   `ros2 launch go2_piper_bringup course1_mapping.launch.py gui:=true world:=<world_path> map_file_path:=/tmp/go2_course1_map.pcd`
   - 默认 world 已设置为 `pick_transport.world`，默认 `mode:=physics`，使用原有 finetune 运控。
   - `enable_teleop:=true` 可手动控制机器人边走边建图。
2. 在地图区域巡航后，按 `Ctrl+C` 关闭 launch。
3. 检查生成：
   - `/tmp/go2_course1_map.pcd` 是否存在。

## 2) 定位+2D导航阶段

1. 启动导航（默认 `mode:=physics`，保留原有 finetune 运控）：
   `ros2 launch go2_piper_bringup course1_nav_2d.launch.py gui:=true world:=$(ros2 pkg prefix go2_piper_bringup)/share/go2_piper_bringup/worlds/pick_transport_obstacle.world map_path:=/tmp/go2_course1_map.pcd`
2. 主要参数：
   - `enable_table_capture:=true`：开启桌边目标触发抓取任务。
   - `enable_mission_server:=true`：启动抓取任务服务。
   - `enable_mission_perception:=true`：启动桌边视觉检测。
   - 导航速度已在移植后的 `go2_config/config/autonomy/navigation.yaml` 固定为约 `0.3 m/s`，用于避障。
   - `use_scan_mid360:=true`：打开点云转 scan 的避障链路。
   - `scan_input_topic:=/cloud_registered`：FAST-LIO 点云来源。
   - `scan_output_topic:=/scan_mid360`：`pointcloud_to_laserscan` 输出话题。

## 3) 触发抓取

- 在 RViz/界面发送导航目标到 `/move_base_simple/goal`。
- 当目标落入桌边矩形区域（默认 x:[0.05,1.10], y:[-0.55,0.45]）时，会自动触发 `PickTransport`。
- 同一区域内、任务未就绪时会继续转发到 Nav2 的 `/goal_pose`，不会破坏原有抓取任务接口。

## 4) 世界文件与参数

- 新增课程世界（含障碍）：`go2_piper_bringup/worlds/pick_transport_obstacle.world`
- 新增启动文件：
  - `course1_mapping.launch.py`
  - `course1_nav_2d.launch.py`
- 新增配置：
  - `fastlio_mapping.yaml`
  - `fastlio_localization.yaml`
  - `open3d_loc_sim.yaml`

## 5) PCD + PGM + YAML 产物说明

- 当前课程链路默认产出：
  - FAST-LIO 建图 PCD：`/tmp/go2_course1_map.pcd`
- 建图结束后可自动生成传统二维地图文件：
  `ros2 run pcd_to_nav2_map convert /tmp/go2_course1_map.pcd --output-prefix /tmp/go2_course1_map/go2_course1_map`
  生成 Nav2 使用的 PGM 和 YAML；导航入口默认读取 `/tmp/go2_course1_map/go2_course1_map.yaml`。
- 该转换器来自 `go2_course1_projects(1)`，使用 Open3D 读取 FAST-LIO 的 PCD，并按课程项目的占据栅格流程处理地面与障碍。
