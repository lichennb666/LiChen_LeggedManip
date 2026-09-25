#include "open3d_loc/pointcloud_conversions.hpp"
#include "open3d_loc/registration.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace open3d_loc
{
namespace
{
Eigen::Matrix4d PoseToMatrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return matrix;
}

Eigen::Matrix4d TransformToMatrix(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaterniond quaternion(
    transform.rotation.w, transform.rotation.x,
    transform.rotation.y, transform.rotation.z);
  if (quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = Eigen::Vector3d(
    transform.translation.x, transform.translation.y,
    transform.translation.z);
  return matrix;
}

geometry_msgs::msg::Pose MatrixToPose(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0));
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

geometry_msgs::msg::Transform MatrixToTransform(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = matrix(0, 3);
  transform.translation.y = matrix(1, 3);
  transform.translation.z = matrix(2, 3);
  Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0));
  quaternion.normalize();
  transform.rotation.x = quaternion.x();
  transform.rotation.y = quaternion.y();
  transform.rotation.z = quaternion.z();
  transform.rotation.w = quaternion.w();
  return transform;
}

Eigen::Matrix4d PoseFromXyzRpyDegrees(const std::vector<double> & values)
{
  if (values.size() != 6U) {
    throw std::invalid_argument("initial_pose must contain x y z roll pitch yaw");
  }
  constexpr double degrees_to_radians = M_PI / 180.0;
  const Eigen::AngleAxisd roll(values[3] * degrees_to_radians, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(values[4] * degrees_to_radians, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(values[5] * degrees_to_radians, Eigen::Vector3d::UnitZ());

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  matrix.block<3, 1>(0, 3) = Eigen::Vector3d(values[0], values[1], values[2]);
  return matrix;
}

bool IsValidTransform(const Eigen::Matrix4d & matrix)
{
  return matrix.allFinite() &&
         std::abs(matrix.block<3, 3>(0, 0).determinant()) > 1.0e-6;
}

std::shared_ptr<PointCloud> LimitPointCount(
  const std::shared_ptr<PointCloud> & cloud, std::size_t maximum)
{
  if (!cloud || maximum == 0U || cloud->points_.size() <= maximum) {
    return cloud;
  }
  const double ratio = static_cast<double>(maximum) /
    static_cast<double>(cloud->points_.size());
  return cloud->RandomDownSample(ratio);
}
}  // namespace

class GlobalLocalizationNode : public rclcpp::Node
{
public:
  GlobalLocalizationNode()
  : Node("open3d_global_localization"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    DeclareAndLoadParameters();

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    // ROS 2 topic tokens cannot begin with a digit, so the ROS 1 reference
    // topic /3dmap becomes /map_3d here.
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map_3d", latched_qos);
    submap_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/submap", latched_qos);
    aligned_scan_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/scan2map", latched_qos);
    base_pose_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/baselink2map", 20);
    odom_pose_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom2map", 20);
    localization_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("/localization_3d", 20);
    confidence_publisher_ =
      create_publisher<std_msgs::msg::Float32>("/localization_3d_confidence", 20);
    delay_publisher_ =
      create_publisher<std_msgs::msg::Float32>("/localization_3d_delay_ms", 20);

    LoadMap();

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GlobalLocalizationNode::OdometryCallback, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GlobalLocalizationNode::ScanCallback, this, std::placeholders::_1));
    initial_pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, 10,
      std::bind(&GlobalLocalizationNode::InitialPoseCallback, this, std::placeholders::_1));

    worker_ = std::thread(&GlobalLocalizationNode::LocalizationLoop, this);
    RCLCPP_INFO(
      get_logger(),
      "Open3D localization ready: map=%zu points, odom=%s, scan=%s",
      map_full_->points_.size(), odom_topic_.c_str(), scan_topic_.c_str());
  }

  ~GlobalLocalizationNode() override
  {
    stop_requested_.store(true);
    wakeup_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void DeclareAndLoadParameters()
  {
    map_path_ = declare_parameter<std::string>("map_path", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/cloud_registered");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");

    queue_size_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("pcd_queue_size", 10)));
    minimum_scan_frames_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>(
          "minimum_scan_frames", static_cast<std::int64_t>(queue_size_))));
    minimum_scan_frames_ = std::min(minimum_scan_frames_, queue_size_);
    localization_interval_ = std::max(
      0.1, declare_parameter<double>("localization_interval", 2.5));
    voxel_size_coarse_ = declare_parameter<double>("voxel_size_coarse", 0.15);
    voxel_size_fine_ = declare_parameter<double>("voxel_size_fine", 0.10);
    initialization_fitness_threshold_ =
      declare_parameter<double>("initialization_fitness_threshold", 0.50);
    tracking_fitness_threshold_ =
      declare_parameter<double>("tracking_fitness_threshold", 0.50);
    required_initial_successes_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("required_initial_successes", 2)));
    maximum_source_points_ = static_cast<std::size_t>(std::max(
        std::int64_t{100}, declare_parameter<std::int64_t>("maximum_source_points", 80000)));
    maximum_target_points_ = static_cast<std::size_t>(std::max(
        std::int64_t{100}, declare_parameter<std::int64_t>("maximum_target_points", 400000)));
    minimum_source_points_ = static_cast<std::size_t>(std::max(
        std::int64_t{20}, declare_parameter<std::int64_t>("minimum_source_points", 300)));
    minimum_target_points_ = static_cast<std::size_t>(std::max(
        std::int64_t{20}, declare_parameter<std::int64_t>("minimum_target_points", 1000)));
    icp_iterations_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("icp_iterations", 30)));
    icp_method_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("icp_method", 1), 0, 2));
    submap_update_distance_ = std::max(
      0.0, declare_parameter<double>("submap_update_distance", 3.5));
    evaluation_distance_multiplier_ = std::max(
      1.0, declare_parameter<double>("evaluation_distance_multiplier", 4.0));
    assume_odom_child_is_base_ =
      declare_parameter<bool>("assume_odom_child_is_base", true);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", true);

    crop_extent_ = declare_parameter<std::vector<double>>(
      "crop_extent", {60.0, 60.0, 40.0});
    initial_icp_scales_ = declare_parameter<std::vector<double>>(
      "initial_icp_scales", {6.0, 4.0, 1.0});
    const auto initial_pose = declare_parameter<std::vector<double>>(
      "initial_pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    if (map_path_.empty()) {
      throw std::runtime_error("parameter 'map_path' must point to a PCD/PLY map");
    }
    if (voxel_size_coarse_ <= 0.0 || voxel_size_fine_ <= 0.0) {
      throw std::runtime_error("voxel sizes must be positive");
    }
    if (crop_extent_.size() != 3U ||
      std::any_of(crop_extent_.begin(), crop_extent_.end(), [](double value) {return value <= 0.0;}))
    {
      throw std::runtime_error("crop_extent must contain three positive values");
    }
    if (initial_icp_scales_.empty()) {
      throw std::runtime_error("initial_icp_scales cannot be empty");
    }

    desired_map_from_base_ = PoseFromXyzRpyDegrees(initial_pose);
    initial_pose_available_ = true;
    initial_guess_pending_ = true;
  }

  void LoadMap()
  {
    map_full_ = std::make_shared<PointCloud>();
    if (!open3d::io::ReadPointCloud(map_path_, *map_full_) || map_full_->IsEmpty()) {
      throw std::runtime_error("failed to read point-cloud map: " + map_path_);
    }
    map_full_->RemoveNonFinitePoints(true, true);
    map_fine_ = map_full_->VoxelDownSample(voxel_size_fine_);
    map_fine_->EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(voxel_size_fine_ * 2.0, 30));

    auto map_visualization = map_full_->VoxelDownSample(voxel_size_coarse_);
    map_visualization->PaintUniformColor({1.0, 0.15, 0.15});
    map_publisher_->publish(Open3DToPointCloud2(
        *map_visualization, map_frame_,
        static_cast<builtin_interfaces::msg::Time>(now())));
  }

  Eigen::Matrix4d LookupTransformMatrix(
    const std::string & target_frame, const std::string & source_frame)
  {
    const auto transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);
    return TransformToMatrix(transform.transform);
  }

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odometry in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }

    Eigen::Matrix4d odom_from_base = PoseToMatrix(message->pose.pose);
    if (!message->child_frame_id.empty() && message->child_frame_id != base_frame_) {
      try {
        odom_from_base *= LookupTransformMatrix(message->child_frame_id, base_frame_);
      } catch (const tf2::TransformException & exception) {
        if (!assume_odom_child_is_base_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Cannot transform odometry child '%s' to '%s': %s",
            message->child_frame_id.c_str(), base_frame_.c_str(), exception.what());
          return;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Using odometry child '%s' as '%s' because their TF is unavailable",
          message->child_frame_id.c_str(), base_frame_.c_str());
      }
    }

    Eigen::Matrix4d map_from_odom;
    bool initialized;
    double fitness;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom_from_base_ = odom_from_base;
      have_odometry_ = true;
      if (initial_pose_available_ && initial_guess_pending_) {
        map_from_odom_ = desired_map_from_base_ * odom_from_base_.inverse();
        initial_guess_pending_ = false;
        initialized_ = false;
        consecutive_initial_successes_ = 0;
      }
      map_from_odom = map_from_odom_;
      initialized = initialized_;
      fitness = localization_fitness_;
    }

    PublishPoses(message->header.stamp, map_from_odom, odom_from_base, initialized, fitness);
    wakeup_.notify_all();
  }

  void ScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    auto cloud = std::make_shared<PointCloud>();
    std::string error;
    if (!PointCloud2ToOpen3D(*message, *cloud, &error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot convert scan to Open3D: %s", error.c_str());
      return;
    }

    const std::string source_frame = message->header.frame_id.empty() ?
      odom_frame_ : message->header.frame_id;
    if (source_frame != odom_frame_) {
      try {
        cloud->Transform(LookupTransformMatrix(odom_frame_, source_frame));
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Cannot transform scan '%s' to '%s': %s",
          source_frame.c_str(), odom_frame_.c_str(), exception.what());
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan_queue_.push_back(std::move(cloud));
      while (scan_queue_.size() > static_cast<std::size_t>(queue_size_)) {
        scan_queue_.pop_front();
      }
    }
    wakeup_.notify_all();
  }

  void InitialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring initial pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      desired_map_from_base_ = PoseToMatrix(message->pose.pose);
      initial_pose_available_ = true;
      initial_guess_pending_ = true;
      initialized_ = false;
      localization_fitness_ = 0.0;
      consecutive_initial_successes_ = 0;
    }
    RCLCPP_INFO(get_logger(), "Received a new initial pose; restarting ICP initialization");
    wakeup_.notify_all();
  }

  void PublishPoses(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Matrix4d & map_from_odom,
    const Eigen::Matrix4d & odom_from_base,
    bool initialized,
    double fitness)
  {
    const Eigen::Matrix4d map_from_base = map_from_odom * odom_from_base;

    nav_msgs::msg::Odometry base_pose;
    base_pose.header.frame_id = map_frame_;
    base_pose.header.stamp = stamp;
    base_pose.child_frame_id = base_frame_;
    base_pose.pose.pose = MatrixToPose(map_from_base);
    base_pose_publisher_->publish(base_pose);

    nav_msgs::msg::Odometry odom_pose;
    odom_pose.header.frame_id = map_frame_;
    odom_pose.header.stamp = stamp;
    odom_pose.child_frame_id = odom_frame_;
    odom_pose.pose.pose = MatrixToPose(map_from_odom);
    odom_pose_publisher_->publish(odom_pose);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom_pose.header;
    transform.child_frame_id = odom_frame_;
    transform.transform = MatrixToTransform(map_from_odom);
    tf_broadcaster_->sendTransform(transform);

    std_msgs::msg::Float32 confidence;
    confidence.data = static_cast<float>(fitness);
    confidence_publisher_->publish(confidence);

    if (initialized) {
      geometry_msgs::msg::PoseStamped localization;
      localization.header = base_pose.header;
      localization.pose = base_pose.pose.pose;
      localization_publisher_->publish(localization);

      const double delay = (now() - rclcpp::Time(stamp)).seconds() * 1000.0;
      std_msgs::msg::Float32 delay_message;
      delay_message.data = static_cast<float>(delay);
      delay_publisher_->publish(delay_message);
    }
  }

  bool SnapshotInputs(
    std::shared_ptr<PointCloud> & scan,
    Eigen::Matrix4d & odom_from_base,
    Eigen::Matrix4d & map_from_odom,
    bool & initialized)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_odometry_ || !initial_pose_available_ || initial_guess_pending_) {
        return false;
      }
      odom_from_base = odom_from_base_;
      map_from_odom = map_from_odom_;
      initialized = initialized_;
    }

    std::deque<std::shared_ptr<PointCloud>> scans;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (scan_queue_.size() < static_cast<std::size_t>(minimum_scan_frames_)) {
        return false;
      }
      scans = scan_queue_;
    }

    scan = std::make_shared<PointCloud>();
    for (const auto & cloud : scans) {
      *scan += *cloud;
    }
    scan->RemoveNonFinitePoints(true, true);
    return !scan->IsEmpty();
  }

  std::shared_ptr<PointCloud> CropMap(const Eigen::Matrix4d & map_from_base)
  {
    const Eigen::Vector3d center = map_from_base.block<3, 1>(0, 3);
    const Eigen::Matrix3d rotation = map_from_base.block<3, 3>(0, 0);
    const Eigen::Vector3d extent(crop_extent_[0], crop_extent_[1], crop_extent_[2]);
    open3d::geometry::OrientedBoundingBox box(center, rotation, extent);
    return map_fine_->Crop(box);
  }

  std::shared_ptr<PointCloud> CropScan(
    const std::shared_ptr<PointCloud> & scan,
    const Eigen::Matrix4d & odom_from_base)
  {
    const Eigen::Vector3d center = odom_from_base.block<3, 1>(0, 3);
    const Eigen::Matrix3d rotation = odom_from_base.block<3, 3>(0, 0);
    const Eigen::Vector3d extent(crop_extent_[0], crop_extent_[1], crop_extent_[2]);
    open3d::geometry::OrientedBoundingBox box(center, rotation, extent);
    return scan->Crop(box);
  }

  void LocalizationLoop()
  {
    auto next_registration = std::chrono::steady_clock::now();
    while (!stop_requested_.load()) {
      {
        std::unique_lock<std::mutex> lock(wakeup_mutex_);
        wakeup_.wait_for(lock, std::chrono::milliseconds(100));
      }
      if (stop_requested_.load()) {
        break;
      }
      const auto current_time = std::chrono::steady_clock::now();
      if (current_time < next_registration) {
        continue;
      }

      std::shared_ptr<PointCloud> scan;
      Eigen::Matrix4d odom_from_base;
      Eigen::Matrix4d map_from_odom;
      bool initialized = false;
      if (!SnapshotInputs(scan, odom_from_base, map_from_odom, initialized)) {
        continue;
      }
      next_registration = current_time +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(localization_interval_));

      const auto start = std::chrono::steady_clock::now();
      auto source = CropScan(scan, odom_from_base)->VoxelDownSample(voxel_size_fine_);
      source = LimitPointCount(source, maximum_source_points_);

      const Eigen::Matrix4d map_from_base = map_from_odom * odom_from_base;
      const Eigen::Vector3d current_submap_center = map_from_base.block<3, 1>(0, 3);
      if (!cached_submap_ ||
        (current_submap_center - cached_submap_center_).norm() > submap_update_distance_)
      {
        cached_submap_ = CropMap(map_from_base);
        cached_submap_center_ = current_submap_center;
      }
      auto target = std::make_shared<PointCloud>(*cached_submap_);
      target = LimitPointCount(target, maximum_target_points_);
      if (source->points_.size() < minimum_source_points_ ||
        target->points_.size() < minimum_target_points_)
      {
        RCLCPP_WARN(
          get_logger(), "Not enough points for registration: source=%zu target=%zu",
          source->points_.size(), target->points_.size());
        continue;
      }

      Eigen::Matrix4d candidate = map_from_odom;
      if (!initialized) {
        auto aligned_by_guess = std::make_shared<PointCloud>(*source);
        aligned_by_guess->Transform(map_from_odom);
        const Eigen::Matrix4d correction = RegistrationMultiScaleIcp(
          aligned_by_guess, target, voxel_size_fine_, icp_method_,
          initial_icp_scales_, icp_iterations_);
        candidate = correction * map_from_odom;
      } else {
        const auto result = RegistrationIcpCorrection(
          source, target, voxel_size_fine_ * 2.0,
          map_from_odom, icp_method_, icp_iterations_);
        candidate = result.transformation_ * map_from_odom;
      }

      if (!IsValidTransform(candidate)) {
        RCLCPP_WARN(get_logger(), "ICP returned an invalid transformation");
        continue;
      }
      const auto evaluation = open3d::pipelines::registration::EvaluateRegistration(
        *source, *target, voxel_size_fine_ * evaluation_distance_multiplier_, candidate);
      const double threshold = initialized ?
        tracking_fitness_threshold_ : initialization_fitness_threshold_;
      const bool accepted = evaluation.fitness_ >= threshold;

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        localization_fitness_ = evaluation.fitness_;
        if (!initialized_) {
          // The reference implementation lets successive initialization ICP
          // passes refine the rough initial guess, even before it reaches the
          // acceptance threshold.
          map_from_odom_ = candidate;
          if (accepted) {
            ++consecutive_initial_successes_;
          } else {
            consecutive_initial_successes_ = 0;
          }
          if (consecutive_initial_successes_ >= required_initial_successes_) {
            initialized_ = true;
          }
        } else if (accepted) {
          map_from_odom_ = candidate;
        }
      }

      if (publish_debug_clouds_) {
        auto aligned = std::make_shared<PointCloud>(*source);
        aligned->Transform(candidate);
        aligned->PaintUniformColor({0.1, 1.0, 0.1});
        target->PaintUniformColor({1.0, 0.55, 0.0});
        const auto stamp = static_cast<builtin_interfaces::msg::Time>(now());
        aligned_scan_publisher_->publish(Open3DToPointCloud2(*aligned, map_frame_, stamp));
        submap_publisher_->publish(Open3DToPointCloud2(*target, map_frame_, stamp));
      }

      const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      RCLCPP_INFO(
        get_logger(),
        "ICP %s: fitness=%.3f rmse=%.3f source=%zu target=%zu cost=%.1f ms%s",
        initialized ? "tracking" : "initialization", evaluation.fitness_,
        evaluation.inlier_rmse_, source->points_.size(), target->points_.size(),
        elapsed, accepted ? " accepted" : " rejected");
    }
  }

  std::string map_path_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string initial_pose_topic_;
  int queue_size_{10};
  int minimum_scan_frames_{10};
  int required_initial_successes_{2};
  int icp_iterations_{30};
  int icp_method_{1};
  double localization_interval_{2.5};
  double voxel_size_coarse_{0.15};
  double voxel_size_fine_{0.10};
  double initialization_fitness_threshold_{0.5};
  double tracking_fitness_threshold_{0.5};
  double submap_update_distance_{3.5};
  double evaluation_distance_multiplier_{4.0};
  std::size_t maximum_source_points_{80000};
  std::size_t maximum_target_points_{400000};
  std::size_t minimum_source_points_{300};
  std::size_t minimum_target_points_{1000};
  bool assume_odom_child_is_base_{true};
  bool publish_debug_clouds_{true};
  std::vector<double> crop_extent_;
  std::vector<double> initial_icp_scales_;

  std::shared_ptr<PointCloud> map_full_;
  std::shared_ptr<PointCloud> map_fine_;

  std::mutex state_mutex_;
  Eigen::Matrix4d odom_from_base_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d map_from_odom_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d desired_map_from_base_{Eigen::Matrix4d::Identity()};
  bool have_odometry_{false};
  bool initial_pose_available_{false};
  bool initial_guess_pending_{false};
  bool initialized_{false};
  int consecutive_initial_successes_{0};
  double localization_fitness_{0.0};

  std::shared_ptr<PointCloud> cached_submap_;
  Eigen::Vector3d cached_submap_center_{Eigen::Vector3d::Zero()};

  std::mutex scan_mutex_;
  std::deque<std::shared_ptr<PointCloud>> scan_queue_;
  std::mutex wakeup_mutex_;
  std::condition_variable wakeup_;
  std::atomic<bool> stop_requested_{false};
  std::thread worker_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr localization_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr delay_publisher_;
};
}  // namespace open3d_loc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<open3d_loc::GlobalLocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("open3d_global_localization"), "%s", exception.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
