#include "livox_laser_simulation/livox_points_plugin.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo_ros/utils.hpp>
#include <ignition/math/Rand.hh>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "livox_laser_simulation/csv_reader.hpp"

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

namespace
{
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr std::uint32_t kPointStep = 32;

template<typename T>
void WriteValue(std::uint8_t * destination, std::size_t offset, const T & value)
{
  std::memcpy(destination + offset, &value, sizeof(T));
}

builtin_interfaces::msg::Time ToRosTime(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const double whole_seconds = std::floor(seconds);
  stamp.sec = static_cast<std::int32_t>(whole_seconds);
  stamp.nanosec = static_cast<std::uint32_t>(
    std::llround((seconds - whole_seconds) * 1.0e9));
  if (stamp.nanosec >= 1000000000U) {
    ++stamp.sec;
    stamp.nanosec -= 1000000000U;
  }
  return stamp;
}
}  // namespace

void LivoxPointsPlugin::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
  ray_sensor_ = parent;
  sdf_ = sdf;
  ros_node_ = gazebo_ros::Node::Get(sdf);

  const std::string csv_uri = sdf->Get<std::string>("csv_file_name");
  if (!LoadScanPattern(csv_uri)) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "Unable to load MID-360 scan pattern: %s",
      csv_uri.c_str());
    return;
  }

  const double sensor_rate = parent->UpdateRate();
  const double point_rate = sdf->Get<double>("point_rate", 200000.0).first;
  const double publish_rate = sdf->Get<double>("publish_rate", 10.0).first;
  downsample_ = std::max<std::size_t>(
    1U, static_cast<std::size_t>(sdf->Get<int>("downsample", 1).first));
  noise_stddev_ = std::max(0.0, sdf->Get<double>("noise_stddev", 0.02).first);

  if (sensor_rate <= 0.0 || point_rate <= 0.0 || publish_rate <= 0.0) {
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "MID-360 update_rate, point_rate and publish_rate must be positive");
    return;
  }

  raw_points_per_batch_ = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::llround(point_rate / sensor_rate)));
  batches_per_frame_ = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::llround(sensor_rate / publish_rate)));

  auto ray_element = sdf->GetElement("ray");
  auto range_element = ray_element->GetElement("range");
  min_range_ = range_element->Get<double>("min");
  max_range_ = range_element->Get<double>("max");

  frame_name_ = gazebo_ros::SensorFrameID(*parent, *sdf);
  point_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/out", rclcpp::SensorDataQoS());

  RayPlugin::Load(parent, sdf);
  auto physics = world->Physics();
  if (physics->GetType() != "ode") {
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "MID-360 simulation requires ODE physics; current engine is %s",
      physics->GetType().c_str());
    return;
  }

  laser_collision_ = physics->CreateCollision("multiray", parent->ParentName());
  laser_collision_->SetName("mid360_ray_sensor_collision");
  laser_collision_->SetRelativePose(parent->Pose());
  laser_collision_->SetInitialRelativePose(parent->Pose());
  ray_shape_.reset(new physics::LivoxOdeMultiRayShape(laser_collision_));
  laser_collision_->SetShape(ray_shape_);
  ray_shape_->Load(sdf);
  ray_shape_->Init();
  InitializeRays();

  const std::size_t expected_points =
    (raw_points_per_batch_ / downsample_) * batches_per_frame_;
  frame_points_.reserve(expected_points);
  parent->SetActive(true);

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "MID-360 ready: %.0f raw points/s, %.1f Hz frames, %zu rays/batch, "
    "downsample=%zu, pattern=%zu directions",
    point_rate, publish_rate, ray_shape_->RayShapes().size(), downsample_,
    scan_pattern_.size());
}

bool LivoxPointsPlugin::LoadScanPattern(const std::string & uri)
{
  std::vector<std::vector<double>> rows;
  const std::string path = ResolveUri(uri);
  if (path.empty() ||
    !livox_laser_simulation::CsvReader::ReadCsvFile(path, rows))
  {
    return false;
  }

  scan_pattern_.clear();
  scan_pattern_.reserve(rows.size());
  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (rows[index].size() != 3U) {
      continue;
    }
    LivoxScanDirection direction;
    direction.azimuth = rows[index][1] * kDegreesToRadians;
    // The CSV zenith is measured from +Z. Gazebo pitches a +X ray around +Y,
    // so subtracting 90 degrees produces the MID-360 -7..52 degree elevation.
    direction.pitch = rows[index][2] * kDegreesToRadians - M_PI_2;
    direction.line = static_cast<std::uint8_t>(index % 4U);
    scan_pattern_.push_back(direction);
  }
  return !scan_pattern_.empty();
}

std::string LivoxPointsPlugin::ResolveUri(const std::string & uri) const
{
  constexpr char prefix[] = "package://";
  if (uri.rfind(prefix, 0) != 0) {
    return uri;
  }

  const std::string resource = uri.substr(sizeof(prefix) - 1U);
  const std::size_t slash = resource.find('/');
  if (slash == std::string::npos) {
    return {};
  }
  try {
    return ament_index_cpp::get_package_share_directory(resource.substr(0, slash)) +
           "/" + resource.substr(slash + 1U);
  } catch (const std::exception &) {
    return {};
  }
}

void LivoxPointsPlugin::InitializeRays()
{
  const auto offset = laser_collision_->RelativePose();
  const std::size_t ray_count =
    (raw_points_per_batch_ + downsample_ - 1U) / downsample_;
  ray_shape_->RayShapes().reserve(ray_count);

  for (std::size_t raw_index = 0; raw_index < raw_points_per_batch_;
    raw_index += downsample_)
  {
    const LivoxScanDirection & direction =
      scan_pattern_[raw_index % scan_pattern_.size()];
    ignition::math::Quaterniond rotation;
    rotation.Euler(0.0, direction.pitch, direction.azimuth);
    const ignition::math::Vector3d axis =
      offset.Rot() * rotation * ignition::math::Vector3d::UnitX;
    ray_shape_->AddRay(
      min_range_ * axis + offset.Pos(), max_range_ * axis + offset.Pos());
  }
}

void LivoxPointsPlugin::UpdateRayDirections(
  std::vector<std::pair<std::size_t, LivoxScanDirection>> & directions)
{
  directions.clear();
  directions.reserve(ray_shape_->RayShapes().size());
  const auto offset = laser_collision_->RelativePose();
  std::size_t ray_index = 0;

  for (std::size_t raw_index = 0; raw_index < raw_points_per_batch_;
    raw_index += downsample_)
  {
    const std::size_t pattern_entry =
      (pattern_index_ + raw_index) % scan_pattern_.size();
    const LivoxScanDirection & direction = scan_pattern_[pattern_entry];
    ignition::math::Quaterniond rotation;
    rotation.Euler(0.0, direction.pitch, direction.azimuth);
    const ignition::math::Vector3d axis =
      offset.Rot() * rotation * ignition::math::Vector3d::UnitX;

    ray_shape_->RayShapes()[ray_index]->SetPoints(
      min_range_ * axis + offset.Pos(), max_range_ * axis + offset.Pos());
    directions.emplace_back(ray_index, direction);
    ++ray_index;
  }
  pattern_index_ = (pattern_index_ + raw_points_per_batch_) % scan_pattern_.size();
}

void LivoxPointsPlugin::AppendCurrentBatch(
  const std::vector<std::pair<std::size_t, LivoxScanDirection>> & directions,
  double acquisition_time)
{
  for (const auto & item : directions) {
    double range = ray_shape_->GetRange(item.first);
    if (range <= min_range_ || range >= max_range_) {
      continue;
    }
    if (noise_stddev_ > 0.0) {
      range += ignition::math::Rand::DblNormal(0.0, noise_stddev_);
    }
    if (range <= min_range_ || range >= max_range_) {
      continue;
    }

    ignition::math::Quaterniond rotation;
    rotation.Euler(0.0, item.second.pitch, item.second.azimuth);
    const ignition::math::Vector3d point =
      range * (rotation * ignition::math::Vector3d::UnitX);

    LivoxPoint output;
    output.x = static_cast<float>(point.X());
    output.y = static_cast<float>(point.Y());
    output.z = static_cast<float>(point.Z());
    output.intensity = static_cast<float>(ray_shape_->GetRetro(item.first));
    output.tag = 0U;
    output.line = item.second.line;
    output.timestamp = acquisition_time;
    frame_points_.push_back(output);
  }
}

void LivoxPointsPlugin::OnNewLaserScans()
{
  if (!ray_shape_ || scan_pattern_.empty()) {
    return;
  }

  const double acquisition_time = world->SimTime().Double();
  if (batches_in_frame_ == 0U) {
    frame_start_time_ = acquisition_time;
    frame_points_.clear();
  }

  std::vector<std::pair<std::size_t, LivoxScanDirection>> directions;
  UpdateRayDirections(directions);
  ray_shape_->Update();
  AppendCurrentBatch(directions, acquisition_time);

  ++batches_in_frame_;
  if (batches_in_frame_ >= batches_per_frame_) {
    PublishFrame();
    batches_in_frame_ = 0U;
  }
}

void LivoxPointsPlugin::PublishFrame()
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_name_;
  message.header.stamp = ToRosTime(frame_start_time_);
  message.height = 1U;
  message.width = static_cast<std::uint32_t>(frame_points_.size());
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = kPointStep;
  message.row_step = message.point_step * message.width;

  message.fields.resize(7U);
  const auto set_field = [&message](
      std::size_t index, const char * name, std::uint32_t offset,
      std::uint8_t datatype) {
      message.fields[index].name = name;
      message.fields[index].offset = offset;
      message.fields[index].datatype = datatype;
      message.fields[index].count = 1U;
    };
  set_field(0, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  set_field(1, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
  set_field(2, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
  set_field(3, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  set_field(4, "tag", 16, sensor_msgs::msg::PointField::UINT8);
  set_field(5, "line", 17, sensor_msgs::msg::PointField::UINT8);
  set_field(6, "timestamp", 24, sensor_msgs::msg::PointField::FLOAT64);

  message.data.assign(message.row_step, 0U);
  for (std::size_t index = 0; index < frame_points_.size(); ++index) {
    std::uint8_t * data = message.data.data() + index * kPointStep;
    const LivoxPoint & point = frame_points_[index];
    WriteValue(data, 0, point.x);
    WriteValue(data, 4, point.y);
    WriteValue(data, 8, point.z);
    WriteValue(data, 12, point.intensity);
    WriteValue(data, 16, point.tag);
    WriteValue(data, 17, point.line);
    WriteValue(data, 24, point.timestamp);
  }
  point_pub_->publish(message);
}
}  // namespace gazebo
