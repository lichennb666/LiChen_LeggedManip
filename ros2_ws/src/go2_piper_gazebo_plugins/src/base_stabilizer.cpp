#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Pose3.hh>
#include <cmath>
#include <functional>
#include <mutex>

namespace go2_piper_gazebo_plugins
{
// Gazebo Classic's contact/damping model is far outside the Isaac/MuJoCo
// training domain of the bundled policy.  This demo-only adapter constrains
// the untrained floating-base modes.  It integrates the same /cmd_vel that is
// sent to the policy so the task-level planar trajectory remains deterministic;
// articulated joints are still driven by the policy and effort controllers.
class BaseStabilizer : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);
    if (sdf->HasElement("target_z")) {
      target_z_ = sdf->Get<double>("target_z");
    }
    command_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = *msg;
        last_command_time_ = world_->SimTime().Double();
      });
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&BaseStabilizer::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double dt = last_update_time_ < 0.0 ? 0.0 : info.simTime.Double() - last_update_time_;
    last_update_time_ = info.simTime.Double();
    geometry_msgs::msg::Twist command;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (info.simTime.Double() - last_command_time_ <= 0.3) {
        command = command_;
      }
    }
    if (!initialized_) {
      const auto initial = model_->WorldPose();
      x_ = initial.Pos().X();
      y_ = initial.Pos().Y();
      yaw_ = initial.Rot().Yaw();
      initialized_ = true;
    }
    x_ += (std::cos(yaw_) * command.linear.x - std::sin(yaw_) * command.linear.y) * dt;
    y_ += (std::sin(yaw_) * command.linear.x + std::cos(yaw_) * command.linear.y) * dt;
    yaw_ += command.angular.z * dt;
    ignition::math::Pose3d pose(x_, y_, target_z_, 0.0, 0.0, yaw_);
    model_->SetWorldPose(pose);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  geometry_msgs::msg::Twist command_;
  std::mutex mutex_;
  double last_update_time_{-1.0};
  double last_command_time_{-1.0};
  double target_z_{0.28};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  bool initialized_{false};
};
GZ_REGISTER_MODEL_PLUGIN(BaseStabilizer)
}  // namespace go2_piper_gazebo_plugins
