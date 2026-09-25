#ifndef OPEN3D_LOC__REGISTRATION_HPP_
#define OPEN3D_LOC__REGISTRATION_HPP_

#include <Eigen/Core>
#include <open3d/Open3D.h>

#include <memory>
#include <vector>

namespace open3d_loc
{
using PointCloud = open3d::geometry::PointCloud;
using RegistrationResult =
  open3d::pipelines::registration::RegistrationResult;

RegistrationResult RegistrationIcpCorrection(
  const std::shared_ptr<PointCloud> & source,
  const std::shared_ptr<PointCloud> & target,
  double correspondence_distance,
  const Eigen::Matrix4d & initial_transform = Eigen::Matrix4d::Identity(),
  int method = 1,
  int iterations = 30);

Eigen::Matrix4d RegistrationMultiScaleIcp(
  const std::shared_ptr<PointCloud> & source,
  const std::shared_ptr<PointCloud> & target,
  double base_voxel_size,
  int method = 1,
  const std::vector<double> & scales = {6.0, 4.0, 1.0},
  int iterations = 30);
}  // namespace open3d_loc

#endif  // OPEN3D_LOC__REGISTRATION_HPP_
