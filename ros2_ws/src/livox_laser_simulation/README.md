# Livox MID-360 simulation for ROS 2

This package ports the Gazebo Classic multi-ray scan-pattern implementation
from Livox's MIT-licensed `livox_laser_simulation` project to ROS 2 Humble.
The MID-360 scan directions come from `scan_mode/mid360.csv` in the reference
project supplied with this workspace.

The plugin publishes Livox PointXYZRTLT-compatible `sensor_msgs/PointCloud2`
data directly. It does not require `livox_ros_driver2` for simulation.

Default model parameters follow the MID-360 nominal specification: 200,000
points/s, 10 Hz frames, 0.1 m blind range, 70 m maximum range, and four laser
lines. Sampling is split into small Gazebo updates so point timestamps represent
simulation acquisition time rather than conversion time.
