#include "franka_joint_limit_failsafe/soft_stop_gravity_controller.hpp"

#include <algorithm>
#include <cassert>
#include <exception>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace franka_joint_limit_failsafe {

controller_interface::InterfaceConfiguration
SoftStopGravityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
SoftStopGravityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn SoftStopGravityController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("soft_limit_margin", 0.18);
    auto_declare<double>("activation_zone_width", 0.10);
    auto_declare<std::vector<double>>("velocity_damping", {5.0, 5.0, 5.0, 5.0, 2.5, 2.5, 2.0});
    auto_declare<std::vector<double>>("limit_stiffness", {55.0, 55.0, 55.0, 60.0, 22.0, 22.0, 18.0});
    auto_declare<std::vector<double>>("inward_damping", {10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 4.0});
    auto_declare<std::vector<double>>("max_limit_torque", {12.0, 12.0, 12.0, 12.0, 7.0, 7.0, 5.0});
  } catch (const std::exception& exc) {
    fprintf(stderr, "Exception during SoftStopGravityController init: %s\n", exc.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SoftStopGravityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  soft_limit_margin_ = get_node()->get_parameter("soft_limit_margin").as_double();
  activation_zone_width_ = get_node()->get_parameter("activation_zone_width").as_double();

  const auto velocity_damping = get_node()->get_parameter("velocity_damping").as_double_array();
  const auto limit_stiffness = get_node()->get_parameter("limit_stiffness").as_double_array();
  const auto inward_damping = get_node()->get_parameter("inward_damping").as_double_array();
  const auto max_limit_torque = get_node()->get_parameter("max_limit_torque").as_double_array();

  if (soft_limit_margin_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "soft_limit_margin must be positive");
    return CallbackReturn::FAILURE;
  }
  if (activation_zone_width_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "activation_zone_width must be positive");
    return CallbackReturn::FAILURE;
  }

  if (velocity_damping.size() != kNumJoints || limit_stiffness.size() != kNumJoints ||
      inward_damping.size() != kNumJoints || max_limit_torque.size() != kNumJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "All gain vectors must have exactly %d entries", kNumJoints);
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < kNumJoints; ++i) {
    velocity_damping_(i) = velocity_damping.at(i);
    limit_stiffness_(i) = limit_stiffness.at(i);
    inward_damping_(i) = inward_damping.at(i);
    max_limit_torque_(i) = std::max(0.0, max_limit_torque.at(i));
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type SoftStopGravityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  update_joint_states();

  for (int i = 0; i < kNumJoints; ++i) {
    const double zone_scale = compute_limit_zone_scale(i);
    const double damping = -zone_scale * velocity_damping_(i) * dq_(i);
    const double limit_torque = compute_limit_torque(i);
    command_interfaces_[i].set_value(damping + limit_torque);
  }

  return controller_interface::return_type::OK;
}

void SoftStopGravityController::update_joint_states() {
  for (int i = 0; i < kNumJoints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

double SoftStopGravityController::compute_limit_torque(int joint_idx) const {
  const double lower_soft_limit = lower_limits_.at(joint_idx) + soft_limit_margin_;
  const double upper_soft_limit = upper_limits_.at(joint_idx) - soft_limit_margin_;
  const double position = q_(joint_idx);
  const double velocity = dq_(joint_idx);
  double torque = 0.0;
  const double zone_scale = compute_limit_zone_scale(joint_idx);

  if (position < lower_soft_limit) {
    const double penetration = lower_soft_limit - position;
    torque += zone_scale * limit_stiffness_(joint_idx) * penetration;
    torque += zone_scale * inward_damping_(joint_idx) * std::max(0.0, -velocity);
  } else if (position > upper_soft_limit) {
    const double penetration = position - upper_soft_limit;
    torque -= zone_scale * limit_stiffness_(joint_idx) * penetration;
    torque -= zone_scale * inward_damping_(joint_idx) * std::max(0.0, velocity);
  }

  const double max_torque = max_limit_torque_(joint_idx);
  return std::clamp(torque, -max_torque, max_torque);
}

double SoftStopGravityController::compute_limit_zone_scale(int joint_idx) const {
  const double lower_soft_limit = lower_limits_.at(joint_idx) + soft_limit_margin_;
  const double upper_soft_limit = upper_limits_.at(joint_idx) - soft_limit_margin_;
  const double position = q_(joint_idx);

  double distance_to_zone = activation_zone_width_;
  if (position < lower_soft_limit) {
    distance_to_zone = 0.0;
  } else if (position > upper_soft_limit) {
    distance_to_zone = 0.0;
  } else {
    distance_to_zone = std::min(position - lower_soft_limit, upper_soft_limit - position);
  }

  if (distance_to_zone >= activation_zone_width_) {
    return 0.0;
  }

  const double normalized = 1.0 - (distance_to_zone / activation_zone_width_);
  return normalized * normalized;
}

}  // namespace franka_joint_limit_failsafe

PLUGINLIB_EXPORT_CLASS(franka_joint_limit_failsafe::SoftStopGravityController,
                       controller_interface::ControllerInterface)
