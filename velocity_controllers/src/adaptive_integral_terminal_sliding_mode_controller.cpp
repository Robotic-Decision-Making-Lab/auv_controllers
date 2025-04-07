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

#include "velocity_controllers/adaptive_integral_terminal_sliding_mode_controller.hpp"

#include <ranges>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace velocity_controllers
{

namespace
{

/// Set the `Twist` message values to NaN.
void reset_twist_msg(geometry_msgs::msg::Twist * msg)  // NOLINT
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  msg->linear.x = nan;
  msg->linear.y = nan;
  msg->linear.z = nan;
  msg->angular.x = nan;
  msg->angular.y = nan;
  msg->angular.z = nan;
}

/// Convert a `Twist` message to a vector of doubles.
auto twist_to_vector(const geometry_msgs::msg::Twist & twist) -> std::vector<double>
{
  return {twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z};
}

/// Apply the sign function using tanh and a given boundary thickness.
auto sign(double x, double thickness) -> double { return std::tanh(x / thickness); }

/// Apply the element-wise sign function to a vector using tanh and a given boundary thickness.
auto sign(const Eigen::Vector6d & x, double thickness) -> Eigen::Vector6d
{
  return x.unaryExpr([thickness](double val) { return sign(val, thickness); });
}

/// Calculate the element-wise error between two vectors, returning NaN if either element is NaN.
auto calculate_error(const std::vector<double> & ref, const std::vector<double> & state) -> std::vector<double>
{
  std::vector<double> error;
  error.reserve(ref.size());
  std::ranges::transform(ref, state, std::back_inserter(error), [](double ref, double state) {
    return !std::isnan(ref) && !std::isnan(state) ? ref - state : std::numeric_limits<double>::quiet_NaN();
  });
  return error;
}

}  // namespace

auto AdaptiveIntegralTerminalSlidingModeController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<adaptive_integral_terminal_sliding_mode_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto AdaptiveIntegralTerminalSlidingModeController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto AdaptiveIntegralTerminalSlidingModeController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();

  dofs_ = params_.joints;
  n_dofs_ = dofs_.size();
  lambda_ = params_.gains.lambda;

  auto get_gain = [this](auto field) {
    auto gain = dofs_ | std::views::transform([&](const auto & dof) { return params_.gains.joints_map[dof].*field; });
    return std::vector<double>(gain.begin(), gain.end());
  };

  // store the controller gains as matrices
  auto k1 = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::k1);
  auto k2 = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::k2);
  auto k1_min = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::k1_min);
  auto mu = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::mu);
  auto alpha = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::alpha);
  auto k_theta = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::k_theta);

  k1_ = Eigen::Vector6d(k1.data()).asDiagonal();
  k2_ = Eigen::Vector6d(k2.data()).asDiagonal();
  alpha_ = Eigen::Vector6d(alpha.data()).asDiagonal();
  k1_min_ = Eigen::Vector6d(k1_min.data());
  mu_ = Eigen::Vector6d(mu.data());
  k_theta_ = Eigen::Vector6d(k_theta_.data());

  const Eigen::Vector3d moments(params_.hydrodynamics.moments_of_inertia.data());
  const Eigen::Vector6d added_mass(params_.hydrodynamics.added_mass.data());
  Eigen::Vector6d linear_damping(params_.hydrodynamics.linear_damping.data());
  Eigen::Vector6d quadratic_damping(params_.hydrodynamics.quadratic_damping.data());
  Eigen::Vector3d cob(params_.hydrodynamics.center_of_buoyancy.data());
  Eigen::Vector3d cog(params_.hydrodynamics.center_of_gravity.data());

  const auto & dyn = params_.hydrodynamics;
  model_ = std::make_unique<hydrodynamics::Params>(
    dyn.mass, moments, added_mass, linear_damping, quadratic_damping, cog, cob, dyn.weight, dyn.buoyancy);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  reference_.writeFromNonRT(geometry_msgs::msg::Twist());
  command_interfaces_.reserve(n_dofs_);
  system_state_values_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());

  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
      reference_.writeFromNonRT(*msg);
    });

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  first_update_ = true;
  system_rotation_.writeFromNonRT(Eigen::Quaterniond::Identity());
  reset_twist_msg(reference_.readFromNonRT());
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::command_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_dofs_);

  for (const auto & dof : dofs_) {
    config.names.emplace_back(
      params_.reference_controller.empty()
        ? std::format("{}/{}", dof, hardware_interface::HW_IF_EFFORT)
        : std::format("{}/{}/{}", params_.reference_controller, dof, hardware_interface::HW_IF_EFFORT));
  }

  return config;
}

auto AdaptiveIntegralTerminalSlidingModeController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_dofs_);

  for (const auto & dof : dofs_) {
    config.names.emplace_back(std::format("{}/{}", dof, hardware_interface::HW_IF_VELOCITY));
  }

  return config;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_export_reference_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());

  for (const auto & dof : dofs_) {
    interfaces.emplace_back(
      get_node()->get_name(),
      std::format("{}/{}", dof, hardware_interface::HW_IF_EFFORT),
      &reference_interfaces_[interfaces.size()]);
  }

  return interfaces;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();
  const std::vector<double> reference = twist_to_vector(*current_reference);
  for (auto && [interface, ref] : std::views::zip(reference_interfaces_, reference)) {
    if (!std::isnan(ref)) {
      interface = ref;
    }
  }
  reset_twist_msg(current_reference);
  return controller_interface::return_type::OK;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_system_state_values() -> controller_interface::return_type
{
  for (auto && [interface, dof, value] : std::views::zip(state_interfaces_, dofs_, system_state_values_)) {
    const auto val = interface.get_optional();
    if (!val.has_value()) {
      // NOLINTNEXTLINE
      RCLCPP_WARN(get_node()->get_logger(), std::format("Failed to get state value for joint {}", dof).c_str());
      return controller_interface::return_type::ERROR;
    }
    value = val.value();
  }
  return controller_interface::return_type::OK;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_and_write_commands(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  configure_parameters();
  update_system_state_values();

  // calculate the velocity error
  const std::vector<double> error_values = calculate_error(reference_interfaces_, system_state_values_);
  if (std::ranges::all_of(error_values, [](double x) { return std::isnan(x); })) {
    // NOLINTNEXTLINE
    RCLCPP_DEBUG(get_node()->get_logger(), "All velocity error values are NaN. Skipping control update.");
    return controller_interface::return_type::OK;
  }

  const Eigen::Vector6d vel(system_state_values_.data());
  const Eigen::Vector6d error(error_values.data());

  // try getting the latest orientation of the vehicle in the inertial frame
  // if the transform is not available, use the last known orientation.
  try {
    const geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform(params_.odom_frame_id, params_.vehicle_frame_id, tf2::TimePointZero);
    tf2::fromMsg(t.transform.rotation, *system_rotation_.readFromRT());
  }
  catch (const tf2::TransformException & e) {
    // NOLINTNEXTLINE
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      std::format(
        "Could not transform {} to {} using latest available transform. {}",
        params_.odom_frame_id,
        params_.vehicle_frame_id,
        e.what())
        .c_str());
  }

  if (first_update_) {
    integral_error_ = -error.array() / alpha_.diagonal().array();
    first_update_ = false;
  }

  // calculate the sliding surface using the current integral error
  const Eigen::Vector6d s = error + alpha_ * integral_error_;

  // calculate the integral error dynamics using p = 3.0, q = 5.0;
  integral_error_ = sign(error, lambda_).asDiagonal() * error.cwiseAbs().array().pow(3.0 / 5.0).matrix();

  // calculate the computed torque control
  // assume that the feedforward acceleration is zero
  const Eigen::Vector6d u = alpha_ * integral_error_;
  const auto & rot = system_rotation_.readFromRT()->toRotationMatrix();
  const Eigen::Vector6d t0 = hydrodynamics::inverse_dynamics(*model_, u, vel, rot);

  // calculate the adaptive disturbance rejection control
  const Eigen::Vector6d t1 = (k1_ * s.cwiseAbs().cwiseSqrt()).asDiagonal() * sign(s, lambda_) + k2_ * s;

  // the total control is the sum of the nominal control and disturbance rejection control
  const Eigen::Vector6d t = t0 + t1;

  for (auto && [command_interface, tau] : std::views::zip(command_interfaces_, t)) {
    if (!command_interface.set_value(tau)) {
      // NOLINTNEXTLINE
      RCLCPP_WARN(
        get_node()->get_logger(),
        std::format("Failed to set command value for joint {}", command_interface.get_name()).c_str());
      return controller_interface::return_type::ERROR;
    };
  }

  // update the adaptive gain
  for (auto [i, val] : std::views::enumerate(k1_.diagonal())) {
    k1_(i, i) = val > k1_min_(i) ? k_theta_(i) * sign(std::abs(val) - mu_(i), lambda_) : k1_min_(i);
  }

  return controller_interface::return_type::OK;
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::AdaptiveIntegralTerminalSlidingModeController,
  controller_interface::ChainableControllerInterface)
