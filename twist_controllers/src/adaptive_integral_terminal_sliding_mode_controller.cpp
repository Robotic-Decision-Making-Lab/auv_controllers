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

#include "twist_controllers/adaptive_integral_terminal_sliding_mode_controller.hpp"

#include <limits>
#include <ranges>
#include <tuple>

#include "controller_common/common.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace twist_controllers
{

namespace
{

/// Apply the sign function using tanh and a given boundary thickness.
auto sign(double x, double thickness) -> double { return std::tanh(x / thickness); }

/// Apply the element-wise sign function to a vector using tanh and a given boundary thickness.
auto sign(const Eigen::Vector6d & x, double thickness) -> Eigen::Vector6d
{
  return x.unaryExpr([thickness](double val) -> double { return sign(val, thickness); });
}

}  // namespace

auto AdaptiveIntegralTerminalSlidingModeController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<adaptive_integral_terminal_sliding_mode_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();

  RCLCPP_INFO(logger_, "Parsing hydrodynamic model from robot description");  // NOLINT
  const auto out = hydrodynamics::parse_model_from_xml(get_robot_description());
  if (!out.has_value()) {
    // NOLINTNEXTLINE
    RCLCPP_ERROR(logger_, "Failed to parse hydrodynamic model from robot description: %s", out.error().c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  model_ = std::make_unique<hydrodynamics::Params>(out.value());

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

  auto get_gain = [this](auto field) -> auto {
    auto gain =
      dofs_ | std::views::transform([&](const auto & dof) -> auto { return params_.gains.joints_map[dof].*field; });
    return std::vector<double>(gain.begin(), gain.end());
  };

  // store the controller gains as matrices
  auto k2 = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::k2);
  auto k1_min = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::k1_min);
  auto mu = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::mu);
  auto alpha = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::alpha);
  auto k_theta = get_gain(&adaptive_integral_terminal_sliding_mode_controller::Params::Gains::MapJoints::k_theta);

  k2_ = Eigen::Vector6d(k2.data()).asDiagonal();
  alpha_ = Eigen::Vector6d(alpha.data()).asDiagonal();
  k1_min_ = Eigen::Vector6d(k1_min.data());
  mu_ = Eigen::Vector6d(mu.data());
  k_theta_ = Eigen::Vector6d(k_theta.data());

  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  reference_.set(geometry_msgs::msg::Twist());
  system_state_.set(nav_msgs::msg::Odometry());

  command_interfaces_.reserve(n_dofs_);
  system_state_values_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());

  // reset the adaptive gain
  k1_ = Eigen::Matrix6d::Identity();

  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/reference",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {  // NOLINT
      reference_.set(*msg);
    });

  // use the tf interface when we aren't getting the state from a topic
  if (params_.use_external_measured_states) {
    RCLCPP_INFO(logger_, "Using external measured states");  // NOLINT
    system_state_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      "~/system_state",
      rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {  // NOLINT
        system_state_.set(*msg);
      });
  } else {
    RCLCPP_INFO(logger_, "Using state interfaces and tf2 to get the system state");  // NOLINT
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  controller_state_pub_ = get_node()->create_publisher<ControllerState>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerState>>(controller_state_pub_);

  controller_state_.dof_states.resize(n_dofs_);
  for (auto && [state, dof] : std::views::zip(controller_state_.dof_states, dofs_)) {
    state.name = dof;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  first_update_ = true;

  system_rotation_ = Eigen::Quaterniond::Identity();

  geometry_msgs::msg::Twist reference;
  common::messages::reset_message(&reference);
  reference_.set(reference);

  nav_msgs::msg::Odometry system_state;
  common::messages::reset_message(&system_state);
  system_state_.set(system_state);

  common::interfaces::fill(ordered_exported_reference_interfaces_, std::numeric_limits<double>::quiet_NaN());
  reference_values_.assign(reference_values_.size(), std::numeric_limits<double>::quiet_NaN());
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
  if (params_.use_external_measured_states) {
    config.type = controller_interface::interface_configuration_type::NONE;
  } else {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(n_dofs_);
    for (const auto & dof : dofs_) {
      config.names.emplace_back(std::format("{}/{}", dof, hardware_interface::HW_IF_VELOCITY));
    }
  }
  return config;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto AdaptiveIntegralTerminalSlidingModeController::on_export_reference_interfaces_list()
  -> std::vector<hardware_interface::CommandInterface::SharedPtr>
{
  reference_values_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  interfaces.reserve(n_dofs_);

  for (const auto & dof : dofs_) {
    interfaces.push_back(
      common::interfaces::make_reference_interface(
        get_node()->get_name(), std::format("{}/{}", dof, hardware_interface::HW_IF_VELOCITY)));
  }

  return interfaces;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  // consume the latest reference so that it is only applied once
  reference_.try_set([this](geometry_msgs::msg::Twist & reference) -> void {
    const std::vector<double> values = common::messages::to_vector(reference);
    for (const auto & [interface, ref] : std::views::zip(ordered_exported_reference_interfaces_, values)) {
      if (!std::isnan(ref)) {
        std::ignore = interface->set_value(ref);
      }
    }
    common::messages::reset_message(&reference);
  });
  return controller_interface::return_type::OK;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto AdaptiveIntegralTerminalSlidingModeController::update_system_state_values() -> controller_interface::return_type
{
  if (params_.use_external_measured_states) {
    system_state_.try_get([this](const nav_msgs::msg::Odometry & state) -> void {
      std::ranges::copy(common::messages::to_vector(state.twist.twist), system_state_values_.begin());
      tf2::fromMsg(state.pose.pose.orientation, system_rotation_);
    });
  } else {
    std::ranges::transform(state_interfaces_, system_state_values_.begin(), [](const auto & interface) -> auto {
      return interface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    });

    try {
      const auto t = tf_buffer_->lookupTransform(params_.odom_frame_id, params_.vehicle_frame_id, tf2::TimePointZero);
      tf2::fromMsg(t.transform.rotation, system_rotation_);
    }
    catch (const tf2::TransformException & e) {
      RCLCPP_DEBUG(  // NOLINT
        logger_,
        std::format("Could not transform {} to {}. {}", params_.odom_frame_id, params_.vehicle_frame_id, e.what())
          .c_str());
    }
  }

  if (common::math::has_nan(system_state_values_)) {
    RCLCPP_DEBUG(logger_, "Received system state with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_and_validate_interfaces()
  -> controller_interface::return_type
{
  if (update_system_state_values() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Failed to update system state values");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  common::interfaces::read_values(ordered_exported_reference_interfaces_, reference_values_);
  if (common::math::has_nan(reference_values_)) {
    RCLCPP_DEBUG(logger_, "Received reference with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) -> controller_interface::return_type
{
  if (update_and_validate_interfaces() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. Failed to update and validate interfaces");  // NOLINT
    return controller_interface::return_type::OK;
  }

  configure_parameters();

  // calculate the velocity error
  const std::vector<double> error_values = common::math::calculate_error(reference_values_, system_state_values_);
  if (common::math::all_nan(error_values)) {
    RCLCPP_DEBUG(logger_, "All velocity error values are NaN. Skipping control update.");  // NOLINT
    return controller_interface::return_type::OK;
  }

  const Eigen::Vector6d vel(system_state_values_.data());
  const Eigen::Vector6d error(error_values.data());

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
  const Eigen::Vector6d u1 = alpha_ * integral_error_;
  const auto rot = system_rotation_.toRotationMatrix();
  const Eigen::Vector6d t0 = hydrodynamics::inverse_dynamics(*model_, u1, vel, rot);

  // calculate the adaptive disturbance rejection control
  const Eigen::Vector6d u2 = (k1_ * s.cwiseAbs().cwiseSqrt()).asDiagonal() * sign(s, lambda_) + k2_ * s;
  const Eigen::Vector6d t1 = model_->M.mass_matrix * u2;

  // the total control is the sum of the nominal control and disturbance rejection control
  const Eigen::Vector6d t = t0 + t1;

  for (auto && [command_interface, tau] : std::views::zip(command_interfaces_, t)) {
    if (!command_interface.set_value(tau)) {
      RCLCPP_WARN(logger_, "Failed to set command for joint %s", command_interface.get_name().c_str());  // NOLINT
    }
  }

  // update the adaptive gain
  for (auto [i, val] : std::views::enumerate(k1_.diagonal())) {
    k1_(i, i) = val > k1_min_(i) ? k_theta_(i) * sign(s(i) - mu_(i), lambda_) : k1_min_(i);
  }

  controller_state_.header.stamp = time;
  for (auto && [i, state] : std::views::enumerate(controller_state_.dof_states)) {
    const auto out = command_interfaces_[i].get_optional();
    state.reference = reference_values_[i];
    state.feedback = system_state_values_[i];
    state.error = error_values[i];
    state.time_step = period.seconds();
    state.output = out.value_or(std::numeric_limits<double>::quiet_NaN());
  }
  rt_controller_state_pub_->try_publish(controller_state_);

  return controller_interface::return_type::OK;
}

}  // namespace twist_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  twist_controllers::AdaptiveIntegralTerminalSlidingModeController,
  controller_interface::ChainableControllerInterface)
