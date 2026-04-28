#pragma once

#include <array>
#include <string>

#include <Eigen/Core>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace franka_joint_limit_failsafe {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SoftStopGravityController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

 private:
  static constexpr int kNumJoints = 7;

  std::string arm_id_;
  Vector7d q_;
  Vector7d dq_;
  Vector7d velocity_damping_;
  Vector7d limit_stiffness_;
  Vector7d inward_damping_;
  Vector7d max_limit_torque_;
  double soft_limit_margin_{0.18};
  double activation_zone_width_{0.10};

  const std::array<double, kNumJoints> lower_limits_{
      {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159}};
  const std::array<double, kNumJoints> upper_limits_{
      {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159}};

  void update_joint_states();
  double compute_limit_torque(int joint_idx) const;
  double compute_limit_zone_scale(int joint_idx) const;
};

}  // namespace franka_joint_limit_failsafe
