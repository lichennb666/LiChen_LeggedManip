#ifndef GO2_PIPER_POLICY_CONTROLLER__POLICY_PD_CONTROLLER_HPP_
#define GO2_PIPER_POLICY_CONTROLLER__POLICY_PD_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace go2_piper_policy_controller
{

struct TargetCommand
{
  std::vector<double> positions;
  std::int64_t stamp_ns{0};
  bool received{false};
};

struct GripperCommand
{
  std::vector<double> positions;
};

class PolicyPdController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool load_contract(const std::string & path);
  void target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void gripper_target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message);

  std::vector<std::string> joints_;
  std::vector<double> default_positions_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;
  std::vector<double> damping_scale_;
  std::vector<double> effort_limits_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> gripper_default_positions_;
  std::vector<double> gripper_stiffness_;
  std::vector<double> gripper_damping_;
  std::vector<double> gripper_effort_limits_;
  std::vector<double> gripper_lower_limits_;
  std::vector<double> gripper_upper_limits_;
  std::vector<double> gripper_previous_positions_;
  std::vector<double> gripper_filtered_velocities_;
  std::vector<double> gripper_setpoints_;
  double gripper_target_slew_rate_{0.04};
  bool gripper_velocity_initialized_{false};
  std::string contract_path_;
  std::string target_topic_;
  std::string gripper_target_topic_;
  double target_timeout_{0.1};
  realtime_tools::RealtimeBuffer<TargetCommand> target_buffer_;
  realtime_tools::RealtimeBuffer<GripperCommand> gripper_target_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    gripper_target_subscription_;
};

}  // namespace go2_piper_policy_controller

#endif  // GO2_PIPER_POLICY_CONTROLLER__POLICY_PD_CONTROLLER_HPP_
