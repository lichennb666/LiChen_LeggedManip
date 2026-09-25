#ifndef OPEN3D_LOC__POINTCLOUD_CONVERSIONS_HPP_
#define OPEN3D_LOC__POINTCLOUD_CONVERSIONS_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <open3d/Open3D.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace open3d_loc
{
bool PointCloud2ToOpen3D(
  const sensor_msgs::msg::PointCloud2 & input,
  open3d::geometry::PointCloud & output,
  std::string * error = nullptr);

sensor_msgs::msg::PointCloud2 Open3DToPointCloud2(
  const open3d::geometry::PointCloud & input,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);
}  // namespace open3d_loc

#endif  // OPEN3D_LOC__POINTCLOUD_CONVERSIONS_HPP_
