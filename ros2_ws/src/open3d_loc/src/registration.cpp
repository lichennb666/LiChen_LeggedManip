#include "open3d_loc/registration.hpp"

#include <algorithm>
#include <stdexcept>

namespace open3d_loc
{
RegistrationResult RegistrationIcpCorrection(
  const std::shared_ptr<PointCloud> & source,
  const std::shared_ptr<PointCloud> & target,
  double correspondence_distance,
  const Eigen::Matrix4d & initial_transform,
  int method,
  int iterations)
{
  if (!source || source->IsEmpty() || !target || target->IsEmpty()) {
    return RegistrationResult();
  }

  auto transformed_source = std::make_shared<PointCloud>(*source);
  transformed_source->Transform(initial_transform);
  const auto criteria = open3d::pipelines::registration::ICPConvergenceCriteria(
    1.0e-6, 1.0e-6, std::max(1, iterations));

  if (method == 0) {
    return open3d::pipelines::registration::RegistrationICP(
      *transformed_source, *target, correspondence_distance,
      Eigen::Matrix4d::Identity(),
      open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
      criteria);
  }
  if (method == 2) {
    return open3d::pipelines::registration::RegistrationGeneralizedICP(
      *transformed_source, *target, correspondence_distance,
      Eigen::Matrix4d::Identity(),
      open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(),
      criteria);
  }
  return open3d::pipelines::registration::RegistrationICP(
    *transformed_source, *target, correspondence_distance,
    Eigen::Matrix4d::Identity(),
    open3d::pipelines::registration::TransformationEstimationPointToPlane(),
    criteria);
}

Eigen::Matrix4d RegistrationMultiScaleIcp(
  const std::shared_ptr<PointCloud> & source,
  const std::shared_ptr<PointCloud> & target,
  double base_voxel_size,
  int method,
  const std::vector<double> & scales,
  int iterations)
{
  if (!source || source->IsEmpty() || !target || target->IsEmpty()) {
    return Eigen::Matrix4d::Identity();
  }
  if (base_voxel_size <= 0.0 || scales.empty()) {
    throw std::invalid_argument("voxel size and ICP scales must be positive");
  }

  Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
  for (const double scale : scales) {
    if (scale <= 0.0) {
      continue;
    }
    const double voxel_size = base_voxel_size * scale;
    auto source_down = source->VoxelDownSample(voxel_size);
    auto target_down = target->VoxelDownSample(voxel_size);
    if (source_down->IsEmpty() || target_down->IsEmpty()) {
      continue;
    }
    if (method == 1 && !target_down->HasNormals()) {
      target_down->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0, 30));
    }

    const auto result = RegistrationIcpCorrection(
      source_down, target_down, voxel_size * 1.5, correction, method, iterations);
    correction = result.transformation_ * correction;
  }
  return correction;
}
}  // namespace open3d_loc
