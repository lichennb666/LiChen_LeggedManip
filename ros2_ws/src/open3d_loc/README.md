# open3d_loc

ROS 2 Humble port of the reference project's PCD-map localization node. It
accumulates FAST-LIO registered scans, crops a local section of the global PCD
map, initializes with multi-scale point-to-plane ICP, and periodically corrects
the `map -> camera_init` transform with fine ICP.

## Interfaces

- Inputs: `/Odometry`, `/cloud_registered`, `/initialpose`
- Pose outputs: `/localization_3d`, `/baselink2map`, `/odom2map`
- Status outputs: `/localization_3d_confidence`, `/localization_3d_delay_ms`
- Debug clouds: `/map_3d`, `/submap`, `/scan2map`
- TF: `map -> camera_init`

The default initial pose is the mapping origin
`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`. Start localization with the robot near the
mapping start position and heading, and the node will initialize automatically.
If that condition cannot be met, set `initial_pose` in
`config/open3d_loc.yaml`, or use RViz's **2D Pose Estimate** tool after startup.
The format is `[x, y, z, roll, pitch, yaw]`, with angles in degrees.

## Run with the MID-360 simulation

Start Gazebo first:

```bash
source scripts/setup_env.sh
ros2 launch go2_config gazebo_mid360.launch.py use_slam:=true
```

Then start FAST-LIO and localization, replacing the map path with a PCD saved
from the mapping run:

```bash
source scripts/setup_env.sh
ros2 launch open3d_loc fast_lio_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd use_sim_time:=true rviz:=true
```

If FAST-LIO is already running, start only the localization node:

```bash
ros2 launch open3d_loc open3d_loc.launch.py \
  map_path:=/absolute/path/to/map.pcd use_sim_time:=true rviz:=false
```

RViz defaults to off to avoid adding load to the simulator. Set `rviz:=true`
only when visualization is needed.

## Main tuning parameters

- `localization_interval`: seconds between ICP passes
- `pcd_queue_size`: number of registered frames accumulated
- `crop_extent`: local map dimensions in metres
- `voxel_size_fine`: tracking resolution
- `initialization_fitness_threshold` / `tracking_fitness_threshold`: minimum
  accepted overlap
- `maximum_source_points` / `maximum_target_points`: computation bounds

The default frames and topics match this workspace's FAST-LIO configuration:
`camera_init`, `base_link`, `/Odometry`, and `/cloud_registered`.

The ROS 1 map topic `/3dmap` was renamed to `/map_3d`, because ROS 2 topic
tokens cannot start with a digit.
