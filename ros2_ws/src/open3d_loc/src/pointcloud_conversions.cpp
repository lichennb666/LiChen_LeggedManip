#include "open3d_loc/pointcloud_conversions.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <exception>

namespace open3d_loc
{
bool PointCloud2ToOpen3D(
  const sensor_msgs::msg::PointCloud2 & input,
  open3d::geometry::PointCloud & output,
  std::string * error)
{
  output.Clear();
  output.points_.reserve(static_cast<std::size_t>(input.width) * input.height);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(input, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
        output.points_.emplace_back(*x, *y, *z);
      }
    }
  } catch (const std::exception & exception) {
    if (error) {
      *error = exception.what();
    }
    output.Clear();
    return false;
  }
  return !output.IsEmpty();
}

sensor_msgs::msg::PointCloud2 Open3DToPointCloud2(
  const open3d::geometry::PointCloud & input,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header.frame_id = frame_id;
  output.header.stamp = stamp;
  output.height = 1U;
  output.is_bigendian = false;
  output.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(input.points_.size());

  sensor_msgs::PointCloud2Iterator<float> x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z(output, "z");
  for (const auto & point : input.points_) {
    *x = static_cast<float>(point.x());
    *y = static_cast<float>(point.y());
    *z = static_cast<float>(point.z());
    ++x;
    ++y;
    ++z;
  }
  return output;
}
}  // namespace open3d_loc
