# pcd_to_nav2_map

Offline conversion of a level PCD/PLY point-cloud map to a Nav2 PGM/YAML map.
The tool does not subscribe to ROS topics and can be run after FAST-LIO exits.

## Build

```bash
cd /home/ubu22/codex_proj/go2_course1_projects/ros2_ws
source scripts/setup_env.sh
colcon build --packages-select pcd_to_nav2_map
source install/setup.bash
```

The Python environment must provide `open3d`, `numpy`, and `Pillow`.

## Example

```bash
ros2 run pcd_to_nav2_map convert \
  --input /absolute/path/map.pcd \
  --resolution 0.05 \
  --z-min -0.10 \
  --z-max 1.00 \
  --free-space-mode flood_fill \
  --seed 0.0 0.0
```

For a point cloud under `FAST_LIO_ROS2`, this creates
`FAST_LIO_ROS2/map/map.pgm`, `map.yaml`, `map_preview.png`, and
`map_stats.json`. The base name follows the input file name. Use `--output`
when a different output prefix is needed.
Pass the YAML—not the PGM—to Nav2:

```bash
ros2 launch go2_config nav2_fastlio.launch.py \
  map:=/absolute/path/nav2_map/map.yaml \
  cloud_topic:=/livox/lidar \
  use_sim_time:=true
```

## Important parameters

- `--resolution`: metres per pixel. `0.05` is the recommended starting point.
- `--z-min`, `--z-max`: map-frame height slice used for static obstacles.
- `--min-points-per-cell`: suppresses isolated projected points.
- `--min-x`, `--max-x`, `--min-y`, `--max-y`: optional crop bounds.
- `--padding`: border outside the point-cloud bounds.
- `--free-space-mode flood_fill`: fills reachable space from one or more seeds.
- `--free-space-mode observed`: marks cells free only when low/ground points exist.
- `--free-space-mode bbox`: treats the complete output rectangle as free except
  for obstacles; convenient but least conservative.
- `--flood-barrier-radius`: closes incomplete wall gaps while flood filling. It
  is specified in metres and defaults to `0.25`. The barrier is used only to
  determine connectivity; reachable cells on the inside are restored afterward.
- `--force`: overwrites an existing output set.

The current FAST-LIO saved map filters most ground points, so `flood_fill` with
the mapping origin `(0, 0)` is the practical default. Always inspect the preview.
For a map that retains ground points, `observed` is more conservative.

Do not independently rotate or translate the 2D map. The PGM/YAML origin is
derived from the point-cloud coordinates so that Open3D localization and Nav2
share the same `map` frame.
