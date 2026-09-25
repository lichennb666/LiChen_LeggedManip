#ifndef LIVOX_LASER_SIMULATION__LIVOX_POINTS_PLUGIN_HPP_
#define LIVOX_LASER_SIMULATION__LIVOX_POINTS_PLUGIN_HPP_

#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "livox_laser_simulation/livox_ode_multiray_shape.hpp"

namespace gazebo
{
struct LivoxScanDirection
{
  double azimuth{0.0};
  double pitch{0.0};
  std::uint8_t line{0};
};

struct LivoxPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  std::uint8_t tag{0};
  std::uint8_t line{0};
  double timestamp{0.0};
};

class LivoxPointsPlugin : public RayPlugin
{
public:
  LivoxPointsPlugin() = default;
  ~LivoxPointsPlugin() override = default;

  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

protected:
  void OnNewLaserScans() override;

private:
  bool LoadScanPattern(const std::string & uri);
  std::string ResolveUri(const std::string & uri) const;
  void InitializeRays();
  void UpdateRayDirections(
    std::vector<std::pair<std::size_t, LivoxScanDirection>> & directions);
  void AppendCurrentBatch(
    const std::vector<std::pair<std::size_t, LivoxScanDirection>> & directions,
    double acquisition_time);
  void PublishFrame();

  boost::shared_ptr<physics::LivoxOdeMultiRayShape> ray_shape_;
  physics::CollisionPtr laser_collision_;
  sensors::SensorPtr ray_sensor_;
  sdf::ElementPtr sdf_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub_;

  std::vector<LivoxScanDirection> scan_pattern_;
  std::vector<LivoxPoint> frame_points_;
  std::size_t pattern_index_{0};
  std::size_t raw_points_per_batch_{1000};
  std::size_t batches_per_frame_{20};
  std::size_t batches_in_frame_{0};
  std::size_t downsample_{1};

  double min_range_{0.1};
  double max_range_{70.0};
  double noise_stddev_{0.02};
  double frame_start_time_{0.0};
  std::string frame_name_{"livox_frame"};
};
}  // namespace gazebo

#endif  // LIVOX_LASER_SIMULATION__LIVOX_POINTS_PLUGIN_HPP_
