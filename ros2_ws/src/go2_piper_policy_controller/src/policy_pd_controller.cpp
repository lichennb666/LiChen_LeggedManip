#include "go2_piper_policy_controller/policy_pd_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "yaml-cpp/yaml.h"

namespace go2_piper_policy_controller
{

controller_interface::CallbackReturn PolicyPdController::on_init()
{
  try {
    auto_declare<std::string>("contract_path", "");
    auto_declare<std::string>("target_topic", "/go2_piper/policy_targets");
    auto_declare<double>("target_timeout", 0.1);
    auto_declare<std::vector<double>>(
      "damping_scale", std::vector<double>(18, 1.0));
    auto_declare<std::vector<std::string>>(
      "gripper_joints", std::vector<std::string>{"joint7", "joint8"});
    auto_declare<std::string>("gripper_target_topic", "/go2_piper/gripper/target");
    auto_declare<std::vector<double>>(
      "gripper_default_positions", std::vector<double>{0.04, -0.04});
    auto_declare<std::vector<double>>(
      "gripper_stiffness", std::vector<double>{100.0, 100.0});
    auto_declare<std::vector<double>>(
      "gripper_damping", std::vector<double>{8.0, 8.0});
    auto_declare<std::vector<double>>(
      "gripper_effort_limits", std::vector<double>{20.0, 20.0});
    auto_declare<std::vector<double>>(
      "gripper_lower_limits", std::vector<double>{0.0, -0.05});
    auto_declare<std::vector<double>>(
      "gripper_upper_limits", std::vector<double>{0.05, 0.0});
    auto_declare<double>("gripper_target_slew_rate", 0.04);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PolicyPdController::load_contract(const std::string & path)
{
  try {
    const YAML::Node contract = YAML::LoadFile(path);
    joints_ = contract["policy_joint_names"].as<std::vector<std::string>>();
    default_positions_ = contract["default_angles"].as<std::vector<double>>();
    stiffness_ = contract["stiffness"].as<std::vector<double>>();
    damping_ = contract["damping"].as<std::vector<double>>();
    effort_limits_ = contract["effort_limits"].as<std::vector<double>>();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "cannot load policy contract %s: %s", path.c_str(), error.what());
    return false;
  }
  const auto size = joints_.size();
  if (size != 18 || default_positions_.size() != size || stiffness_.size() != size ||
      damping_.size() != size || effort_limits_.size() != size) {
    RCLCPP_ERROR(get_node()->get_logger(), "policy contract vectors must all have 18 entries");
    return false;
  }
  for (std::size_t index = 0; index < size; ++index) {
    if (!std::isfinite(default_positions_[index]) || stiffness_[index] <= 0.0 ||
        damping_[index] < 0.0 || effort_limits_[index] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "invalid policy contract value at index %zu", index);
      return false;
    }
  }
  return true;
}

controller_interface::InterfaceConfiguration
PolicyPdController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  for (const auto & joint : gripper_joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
PolicyPdController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : gripper_joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint : gripper_joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::CallbackReturn PolicyPdController::on_configure(
  const rclcpp_lifecycle::State &)
{
  contract_path_ = get_node()->get_parameter("contract_path").as_string();
  target_topic_ = get_node()->get_parameter("target_topic").as_string();
  target_timeout_ = get_node()->get_parameter("target_timeout").as_double();
  damping_scale_ = get_node()->get_parameter("damping_scale").as_double_array();
  gripper_joints_ = get_node()->get_parameter("gripper_joints").as_string_array();
  gripper_target_topic_ = get_node()->get_parameter("gripper_target_topic").as_string();
  gripper_default_positions_ =
    get_node()->get_parameter("gripper_default_positions").as_double_array();
  gripper_stiffness_ = get_node()->get_parameter("gripper_stiffness").as_double_array();
  gripper_damping_ = get_node()->get_parameter("gripper_damping").as_double_array();
  gripper_effort_limits_ =
    get_node()->get_parameter("gripper_effort_limits").as_double_array();
  gripper_lower_limits_ =
    get_node()->get_parameter("gripper_lower_limits").as_double_array();
  gripper_upper_limits_ =
    get_node()->get_parameter("gripper_upper_limits").as_double_array();
  gripper_target_slew_rate_ =
    get_node()->get_parameter("gripper_target_slew_rate").as_double();
  if (contract_path_.empty() || target_timeout_ <= 0.0 || !load_contract(contract_path_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "invalid PD controller configuration");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (damping_scale_.size() != joints_.size() ||
      std::any_of(damping_scale_.begin(), damping_scale_.end(),
        [](double value) { return value < 0.0; })) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "damping_scale must have 18 non-negative entries");
    return controller_interface::CallbackReturn::ERROR;
  }
  const std::size_t gripper_count = gripper_joints_.size();
  if (gripper_count != 2 || gripper_default_positions_.size() != gripper_count ||
      gripper_stiffness_.size() != gripper_count ||
      gripper_damping_.size() != gripper_count ||
      gripper_effort_limits_.size() != gripper_count ||
      gripper_lower_limits_.size() != gripper_count ||
      gripper_upper_limits_.size() != gripper_count) {
    RCLCPP_ERROR(get_node()->get_logger(), "gripper controller vectors must all have 2 entries");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < gripper_count; ++index) {
    if (gripper_stiffness_[index] <= 0.0 || gripper_damping_[index] < 0.0 ||
        gripper_effort_limits_[index] <= 0.0 ||
        gripper_lower_limits_[index] >= gripper_upper_limits_[index] ||
        gripper_default_positions_[index] < gripper_lower_limits_[index] ||
      gripper_default_positions_[index] > gripper_upper_limits_[index] ||
      gripper_target_slew_rate_ <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "invalid gripper controller value at index %zu", index);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  target_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    target_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&PolicyPdController::target_callback, this, std::placeholders::_1));
  gripper_target_subscription_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    gripper_target_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&PolicyPdController::gripper_target_callback, this, std::placeholders::_1));
  TargetCommand initial;
  initial.positions = default_positions_;
  target_buffer_.initRT(initial);
  GripperCommand gripper_initial;
  gripper_initial.positions = gripper_default_positions_;
  gripper_target_buffer_.initRT(gripper_initial);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "loaded contract with %zu policy + %zu gripper joints; target topics=%s,%s",
    joints_.size(), gripper_joints_.size(), target_topic_.c_str(), gripper_target_topic_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PolicyPdController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const std::size_t total_count = joints_.size() + gripper_joints_.size();
  if (command_interfaces_.size() != total_count ||
      state_interfaces_.size() != 2 * total_count) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "claimed interface count mismatch: command=%zu state=%zu",
      command_interfaces_.size(), state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < total_count; ++index) {
    const std::string expected_joint =
      index < joints_.size() ? joints_[index] : gripper_joints_[index - joints_.size()];
    const std::string expected_command =
      expected_joint + "/" + hardware_interface::HW_IF_EFFORT;
    const std::string expected_position =
      expected_joint + "/" + hardware_interface::HW_IF_POSITION;
    const std::string expected_velocity =
      expected_joint + "/" + hardware_interface::HW_IF_VELOCITY;
    if (command_interfaces_[index].get_name() != expected_command ||
        state_interfaces_[index].get_name() != expected_position ||
        state_interfaces_[total_count + index].get_name() != expected_velocity) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "interface ordering mismatch at %zu: command=%s position=%s velocity=%s",
        index, command_interfaces_[index].get_name().c_str(),
        state_interfaces_[index].get_name().c_str(),
        state_interfaces_[total_count + index].get_name().c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
  TargetCommand initial;
  initial.positions = default_positions_;
  target_buffer_.initRT(initial);
  GripperCommand gripper_initial;
  gripper_initial.positions.resize(gripper_joints_.size());
  for (std::size_t index = 0; index < gripper_joints_.size(); ++index) {
    gripper_initial.positions[index] =
      state_interfaces_[joints_.size() + index].get_value();
  }
  gripper_target_buffer_.initRT(gripper_initial);
  gripper_previous_positions_.resize(gripper_joints_.size());
  gripper_filtered_velocities_.assign(gripper_joints_.size(), 0.0);
  gripper_setpoints_ = gripper_initial.positions;
  for (std::size_t index = 0; index < gripper_joints_.size(); ++index) {
    gripper_previous_positions_[index] =
      state_interfaces_[joints_.size() + index].get_value();
  }
  gripper_velocity_initialized_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PolicyPdController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void PolicyPdController::target_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
  if (message->data.size() != joints_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "rejecting policy target with %zu values; expected %zu",
      message->data.size(), joints_.size());
    return;
  }
  TargetCommand command;
  command.positions = message->data;
  command.stamp_ns = get_node()->get_clock()->now().nanoseconds();
  command.received = true;
  target_buffer_.writeFromNonRT(command);
}

void PolicyPdController::gripper_target_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
  if (message->data.size() != gripper_joints_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "rejecting gripper target with %zu values; expected %zu",
      message->data.size(), gripper_joints_.size());
    return;
  }
  GripperCommand command;
  command.positions.resize(gripper_joints_.size());
  for (std::size_t index = 0; index < gripper_joints_.size(); ++index) {
    if (!std::isfinite(message->data[index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "rejecting non-finite gripper target");
      return;
    }
    command.positions[index] = std::clamp(
      message->data[index], gripper_lower_limits_[index], gripper_upper_limits_[index]);
  }
  gripper_target_buffer_.writeFromNonRT(command);
}

controller_interface::return_type PolicyPdController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const TargetCommand * command = target_buffer_.readFromRT();
  const bool fresh = command->received &&
    (time.nanoseconds() - command->stamp_ns) <=
    static_cast<std::int64_t>(target_timeout_ * 1.0e9);
  const std::vector<double> & target = fresh ? command->positions : default_positions_;

  const std::size_t count = joints_.size();
  const std::size_t total_count = count + gripper_joints_.size();
  for (std::size_t index = 0; index < count; ++index) {
    const double position = state_interfaces_[index].get_value();
    const double velocity = state_interfaces_[total_count + index].get_value();
    if (!std::isfinite(position) || !std::isfinite(velocity)) {
      command_interfaces_[index].set_value(0.0);
      return controller_interface::return_type::ERROR;
    }
    const double raw_effort =
      stiffness_[index] * (target[index] - position)
      - damping_[index] * damping_scale_[index] * velocity;
    const double effort = std::clamp(
      raw_effort, -effort_limits_[index], effort_limits_[index]);
    command_interfaces_[index].set_value(effort);
  }
  const GripperCommand * gripper_command = gripper_target_buffer_.readFromRT();
  for (std::size_t gripper_index = 0;
    gripper_index < gripper_joints_.size(); ++gripper_index)
  {
    const std::size_t interface_index = count + gripper_index;
    const double position = state_interfaces_[interface_index].get_value();
    const double reported_velocity =
      state_interfaces_[total_count + interface_index].get_value();
    if (!std::isfinite(position) || !std::isfinite(reported_velocity)) {
      command_interfaces_[interface_index].set_value(0.0);
      return controller_interface::return_type::ERROR;
    }
    const double dt = period.seconds();
    double filtered_velocity = 0.0;
    if (gripper_velocity_initialized_ && dt > 1.0e-6 && dt < 0.1) {
      const double finite_difference =
        (position - gripper_previous_positions_[gripper_index]) / dt;
      constexpr double velocity_filter_alpha = 1.0;
      gripper_filtered_velocities_[gripper_index] =
        velocity_filter_alpha * finite_difference +
        (1.0 - velocity_filter_alpha) * gripper_filtered_velocities_[gripper_index];
      filtered_velocity = gripper_filtered_velocities_[gripper_index];
    }
    gripper_previous_positions_[gripper_index] = position;
    const double max_setpoint_step = gripper_target_slew_rate_ * std::max(dt, 0.0);
    const double setpoint_error =
      gripper_command->positions[gripper_index] - gripper_setpoints_[gripper_index];
    gripper_setpoints_[gripper_index] += std::clamp(
      setpoint_error, -max_setpoint_step, max_setpoint_step);
    const double raw_effort =
      gripper_stiffness_[gripper_index] *
      (gripper_setpoints_[gripper_index] - position) -
      gripper_damping_[gripper_index] * filtered_velocity;
    double effort = std::clamp(
      raw_effort, -gripper_effort_limits_[gripper_index],
      gripper_effort_limits_[gripper_index]);
    constexpr double limit_guard = 1.0e-4;
    if ((position <= gripper_lower_limits_[gripper_index] + limit_guard && effort < 0.0) ||
        (position >= gripper_upper_limits_[gripper_index] - limit_guard && effort > 0.0)) {
      effort = 0.0;
    }
    command_interfaces_[interface_index].set_value(effort);
  }
  return controller_interface::return_type::OK;
}

}  // namespace go2_piper_policy_controller

PLUGINLIB_EXPORT_CLASS(
  go2_piper_policy_controller::PolicyPdController,
  controller_interface::ControllerInterface)
