// Copyright 2025, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "impedance_controller/impedance_controller.hpp"

#include <ranges>
#include <unsupported/Eigen/MatrixFunctions>

#include "controller_common/common.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace impedance_controller
{

namespace
{

auto vee(const Eigen::Matrix4d & mat) -> Eigen::Vector6d
{
  return {mat(0, 3), mat(1, 3), mat(2, 3), mat(2, 1), mat(0, 2), mat(1, 0)};
}

auto geodesic_error(const geometry_msgs::msg::Pose & goal, const geometry_msgs::msg::Pose & state) -> Eigen::Vector6d
{
  Eigen::Isometry3d goal_mat, state_mat;  // NOLINT
  tf2::fromMsg(goal, goal_mat);
  tf2::fromMsg(state, state_mat);
  const Eigen::Matrix4d error = (goal_mat.inverse() * state_mat).matrix().log();
  return vee(error);
}

}  // namespace

auto ImpedanceController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<impedance_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();
  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto ImpedanceController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();

  command_dofs_ = params_.command_joints;
  n_command_dofs_ = command_dofs_.size();

  state_dofs_ = params_.state_joints;
  n_state_dofs_ = state_dofs_.size();
  n_reference_dofs_ = n_command_dofs_ + n_state_dofs_;

  auto get_gains = [this](auto field) {
    auto gains = command_dofs_ |
                 std::views::transform([&](const auto & dof) { return params_.gains.command_joints_map[dof].*field; });
    return std::vector<double>(gains.begin(), gains.end());
  };

  // proportional gain
  auto kp = get_gains(&impedance_controller::Params::Gains::MapCommandJoints::kp);
  kp_ = Eigen::Vector6d(kp.data()).asDiagonal();

  // derivative gain
  auto kd = get_gains(&impedance_controller::Params::Gains::MapCommandJoints::kd);
  kd_ = Eigen::Vector6d(kd.data()).asDiagonal();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  reference_.writeFromNonRT(auv_control_msgs::msg::ImpedanceCommand());
  system_state_.writeFromNonRT(nav_msgs::msg::Odometry());

  command_interfaces_.reserve(n_command_dofs_);
  state_interfaces_.reserve(n_state_dofs_);

  system_state_values_.resize(n_state_dofs_, std::numeric_limits<double>::quiet_NaN());

  if (params_.use_external_measured_states) {
    RCLCPP_INFO(logger_, "Using external measured states");  // NOLINT
    system_state_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      "~/system_state",
      rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {  // NOLINT
        system_state_.writeFromNonRT(*msg);
      });
  }

  controller_state_pub_ = get_node()->create_publisher<ControllerState>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerState>>(controller_state_pub_);

  controller_state_.dof_states.resize(n_command_dofs_);
  for (auto && [state, dof] : std::views::zip(controller_state_.dof_states, command_dofs_)) {
    state.name = dof;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  common::messages::reset_message(reference_.readFromNonRT());
  common::messages::reset_message(system_state_.readFromNonRT());

  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::command_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_command_dofs_);

  for (const auto & dof : command_dofs_) {
    config.names.emplace_back(
      params_.reference_controller.empty()
        ? std::format("{}/{}", dof, hardware_interface::HW_IF_EFFORT)
        : std::format("{}/{}/{}", params_.reference_controller, dof, hardware_interface::HW_IF_EFFORT));
  }

  return config;
}

auto ImpedanceController::state_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  if (params_.use_external_measured_states) {
    config.type = controller_interface::interface_configuration_type::NONE;
  } else {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(n_state_dofs_);
    const std::size_t n_pose_dofs = 7;
    for (std::size_t i = 0; i < n_pose_dofs; ++i) {
      config.names.emplace_back(std::format("{}/{}", state_dofs_[i], hardware_interface::HW_IF_POSITION));
    }
    for (std::size_t i = n_pose_dofs; i < n_state_dofs_; ++i) {
      config.names.emplace_back(std::format("{}/{}", state_dofs_[i], hardware_interface::HW_IF_VELOCITY));
    }
  }
  return config;
}

auto ImpedanceController::on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  // the reference command includes the desired pose, velocity, and force/torque
  reference_interfaces_.resize(n_reference_dofs_, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());

  // add the pose & velocity interfaces
  // this uses the same position/velocity joint names as the state interfaces
  for (const auto [i, dof] : std::views::enumerate(state_dofs_)) {
    interfaces.emplace_back(
      get_node()->get_name(), std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION), &reference_interfaces_[i]);
  }

  // add the force/torque interfaces
  // this uses the same effort joint names as the command interfaces
  for (const auto [i, dof] : std::views::enumerate(command_dofs_)) {
    interfaces.emplace_back(
      get_node()->get_name(),
      std::format("{}/{}", dof, hardware_interface::HW_IF_EFFORT),
      &reference_interfaces_[i + n_state_dofs_]);
  }

  return interfaces;
}

auto ImpedanceController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();
  const std::vector<double> reference = common::messages::to_vector(*current_reference);
  for (auto && [interface, ref] : std::views::zip(reference_interfaces_, reference)) {
    if (!std::isnan(ref)) {
      interface = ref;
    }
  }
  common::messages::reset_message(current_reference);
  return controller_interface::return_type::OK;
}

auto ImpedanceController::update_system_state_values() -> controller_interface::return_type
{
  if (params_.use_external_measured_states) {
    auto * current_state = system_state_.readFromRT();
    std::ranges::copy(common::messages::to_vector(*current_state), system_state_values_.begin());
  } else {
    std::ranges::transform(state_interfaces_, system_state_values_.begin(), [](const auto & interface) {
      return interface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    });
  }

  if (common::math::has_nan(system_state_values_)) {
    RCLCPP_DEBUG(logger_, "Received system state with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

auto ImpedanceController::update_and_validate_interfaces() -> controller_interface::return_type
{
  if (update_system_state_values() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Failed to update system state values");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  if (common::math::has_nan(reference_interfaces_)) {
    RCLCPP_DEBUG(logger_, "Received reference with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

auto ImpedanceController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  if (update_and_validate_interfaces() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. Failed to update and validate interfaces");  // NOLINT
    return controller_interface::return_type::OK;
  }

  configure_parameters();

  // get the reference pose, twist, and wrench values
  const auto pose_end = reference_interfaces_.begin() + 7;
  const auto twist_end = pose_end + 6;
  const auto wrench_end = twist_end + 6;

  const std::vector<double> ref_pose_values(reference_interfaces_.begin(), pose_end);
  const std::vector<double> ref_twist_values(pose_end, twist_end);
  const std::vector<double> ref_wrench_values(twist_end, wrench_end);

  // get the state pose and twist values
  const std::vector<double> state_pose_values(system_state_values_.begin(), system_state_values_.begin() + 7);
  const std::vector<double> state_twist_values(system_state_values_.begin() + 7, system_state_values_.end());

  // get the state and reference pose as messages
  geometry_msgs::msg::Pose reference_pose;
  common::messages::to_msg(ref_pose_values, &reference_pose);

  geometry_msgs::msg::Pose state_pose;
  common::messages::to_msg(state_pose_values, &state_pose);

  // calculate the pose error
  const Eigen::Vector6d pose_error = geodesic_error(reference_pose, state_pose);
  const std::vector<double> pose_error_values(pose_error.data(), pose_error.data() + pose_error.size());

  // calculate the velocity error
  const std::vector<double> twist_error_values = common::math::calculate_error(ref_twist_values, state_twist_values);
  const Eigen::Vector6d twist_error(twist_error_values.data());

  if (common::math::all_nan(pose_error_values)) {
    RCLCPP_DEBUG(logger_, "All pose error values are NaN. Skipping control update.");  // NOLINT
    return controller_interface::return_type::OK;
  }

  if (common::math::all_nan(twist_error_values)) {
    RCLCPP_DEBUG(logger_, "All velocity error values are NaN. Skipping control update.");  // NOLINT
    return controller_interface::return_type::OK;
  }

  // convert the reference wrench values into an Eigen vector
  Eigen::Vector6d reference_wrench(ref_wrench_values.data());

  // calculate the control command
  Eigen::Vector6d t = reference_wrench + kp_ * pose_error + kd_ * twist_error;

  for (auto && [command_interface, tau] : std::views::zip(command_interfaces_, t)) {
    if (!command_interface.set_value(tau)) {
      RCLCPP_WARN(logger_, "Failed to set command for joint %s", command_interface.get_name().c_str());  // NOLINT
    }
  }

  controller_state_.header.stamp = time;
  for (auto && [i, state] : std::views::enumerate(controller_state_.dof_states)) {
    const auto out = command_interfaces_[i].get_optional();
    state.error = pose_error_values[i];
    state.error_dot = twist_error_values[i];
    state.time_step = period.seconds();
    state.output = out.value_or(std::numeric_limits<double>::quiet_NaN());
  }

  // the feedback and reference values have different sizes than the command interfaces
  for (std::size_t i = 0; i < n_state_dofs_; ++i) {
    controller_state_.dof_states[i].feedback = system_state_values_[i];
  }
  for (std::size_t i = n_state_dofs_; i < n_reference_dofs_; ++i) {
    controller_state_.dof_states[i].reference = reference_interfaces_[i];
  }

  rt_controller_state_pub_->try_publish(controller_state_);

  return controller_interface::return_type::OK;
}

}  // namespace impedance_controller
