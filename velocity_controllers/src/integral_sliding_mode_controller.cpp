// Copyright 2024, Evan Palmer
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

#include "velocity_controllers/integral_sliding_mode_controller.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <iterator>
#include <limits>
#include <ranges>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace velocity_controllers
{

namespace
{

void reset_twist_msg(geometry_msgs::msg::Twist * msg)  // NOLINT
{
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}

void reset_odom_msg(nav_msgs::msg::Odometry * msg)  // NOLINT
{
  msg->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond::Identity());
  reset_twist_msg(&msg->twist.twist);
}

auto twist_to_vector(const geometry_msgs::msg::Twist & twist) -> std::vector<double>
{
  return {twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z};
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

auto IntegralSlidingModeController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<integral_sliding_mode_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IntegralSlidingModeController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto IntegralSlidingModeController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();

  dofs_ = params_.joints;
  n_dofs_ = dofs_.size();
  boundary_thickness_ = params_.gains.lambda;

  auto get_gain = [this](auto field) {
    auto gain = dofs_ | std::views::transform([&](const auto & dof) { return params_.gains.joints_map[dof].*field; });
    return std::vector<double>(gain.begin(), gain.end());
  };

  auto rho = get_gain(&integral_sliding_mode_controller::Params::Gains::MapJoints::rho);
  auto kp = get_gain(&integral_sliding_mode_controller::Params::Gains::MapJoints::kp);

  rho_ = Eigen::Vector6d(rho.data()).asDiagonal().toDenseMatrix();
  kp_ = Eigen::Vector6d(kp.data()).asDiagonal().toDenseMatrix();

  vehicle_frame_id_ = params_.vehicle_frame_id;
  inertial_frame_id_ = params_.odom_frame_id;

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  reference_.writeFromNonRT(geometry_msgs::msg::Twist());
  system_state_.writeFromNonRT(nav_msgs::msg::Odometry());

  command_interfaces_.reserve(n_dofs_);
  system_state_values_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());

  // NOLINTBEGIN
  RCLCPP_INFO(get_node()->get_logger(), "Commands won't be sent until both reference and state messages are received.");
  RCLCPP_INFO(get_node()->get_logger(), "Waiting for robot_description to be received");
  // NOLINTEND

  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
      reference_.writeFromNonRT(*msg);
    });

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "robot_description", qos, [this](const std::shared_ptr<std_msgs::msg::String> msg) {
      if (model_initialized_ || msg->data.empty()) {
        return;
      }
      RCLCPP_INFO(get_node()->get_logger(), "Parsing hydrodynamic model from robot description");  // NOLINT
      const auto out = hydrodynamics::parse_model_from_xml(msg->data);
      if (!out.has_value()) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          std::format("Failed to parse hydrodynamic model from robot description: {}", out.error()).c_str());
        return;
      }
      model_ = std::make_unique<hydrodynamics::Params>(out.value());
      model_initialized_ = true;
    });

  // use the tf interface when we aren't getting the state from a topic
  if (params_.use_external_measured_states) {
    RCLCPP_INFO(get_node()->get_logger(), "Using external measured states");  // NOLINT
    system_state_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      "~/system_state", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        system_state_.writeFromNonRT(*msg);
      });
  } else {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  controller_state_pub_ =
    get_node()->create_publisher<control_msgs::msg::MultiDOFStateStamped>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>>(controller_state_pub_);

  rt_controller_state_pub_->lock();
  rt_controller_state_pub_->msg_.dof_states.resize(n_dofs_);
  for (auto && [state, dof] : std::views::zip(rt_controller_state_pub_->msg_.dof_states, dofs_)) {
    state.name = dof;
  }
  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  first_update_ = true;
  total_error_ = Eigen::Vector6d::Zero();
  init_error_ = Eigen::Vector6d::Zero();

  system_rotation_.writeFromNonRT(Eigen::Quaterniond::Identity());
  reset_twist_msg(reference_.readFromNonRT());
  reset_odom_msg(system_state_.readFromNonRT());

  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::command_interface_configuration() const
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

auto IntegralSlidingModeController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = params_.use_external_measured_states ? controller_interface::interface_configuration_type::NONE
                                                     : controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_dofs_);

  for (const auto & dof : dofs_) {
    config.names.emplace_back(std::format("{}/{}", dof, hardware_interface::HW_IF_VELOCITY));
  }

  return config;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IntegralSlidingModeController::on_export_reference_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());

  for (const auto [i, dof] : std::views::enumerate(dofs_)) {
    interfaces.emplace_back(
      get_node()->get_name(), std::format("{}/{}", dof, hardware_interface::HW_IF_VELOCITY), &reference_interfaces_[i]);
  }

  return interfaces;
}

auto IntegralSlidingModeController::update_reference_from_subscribers(
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

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IntegralSlidingModeController::update_system_state_values() -> controller_interface::return_type
{
  if (params_.use_external_measured_states) {
    auto * current_state = system_state_.readFromNonRT();
    const std::vector<double> state = twist_to_vector(current_state->twist.twist);
    system_state_values_.assign(state.begin(), state.end());
    tf2::fromMsg(current_state->pose.pose.orientation, *system_rotation_.readFromRT());
  } else {
    // read the velocity values from the state interfaces
    for (auto && [interface, dof, value] : std::views::zip(state_interfaces_, dofs_, system_state_values_)) {
      const auto val = interface.get_optional();
      if (!val.has_value()) {
        // NOLINTNEXTLINE
        RCLCPP_WARN(get_node()->get_logger(), std::format("Failed to get state value for joint {}", dof).c_str());
        return controller_interface::return_type::ERROR;
      }
      value = val.value();
    }

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
  }

  return controller_interface::return_type::OK;
}

auto IntegralSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) -> controller_interface::return_type
{
  if (!model_initialized_) {
    return controller_interface::return_type::OK;
  }

  configure_parameters();
  update_system_state_values();
  const Eigen::Vector6d velocity_state(system_state_values_.data());

  // Calculate the velocity error
  const std::vector<double> error_values = calculate_error(reference_interfaces_, system_state_values_);
  if (std::ranges::all_of(error_values, [](double x) { return std::isnan(x); })) {
    RCLCPP_DEBUG(get_node()->get_logger(), "All velocity error values are NaN. Skipping control update.");  // NOLINT
    return controller_interface::return_type::OK;
  }

  const Eigen::Vector6d vel(system_state_values_.data());
  const Eigen::Vector6d error(error_values.data());

  if (first_update_) {
    init_error_ = error;
    first_update_ = false;
  } else {
    total_error_ += error * period.seconds();
  }

  // calculate the computed torque control
  // assume that the feedforward acceleration is zero
  const Eigen::Vector6d u = kp_ * error;
  const auto & rot = system_rotation_.readFromRT()->toRotationMatrix();
  const Eigen::Vector6d tau0 = hydrodynamics::inverse_dynamics(*model_, u, vel, rot);

  // calculate the disturbance rejection torque
  Eigen::Vector6d surface = error + kp_ * total_error_ - kp_ * init_error_;
  surface = surface.unaryExpr([this](double x) { return std::tanh(x / boundary_thickness_); });
  const Eigen::Vector6d tau1 = rho_ * surface;

  // total control torque
  const Eigen::Vector6d tau = tau0 + tau1;

  for (std::size_t i = 0; i < n_dofs_; ++i) {
    if (!command_interfaces_[i].set_value(tau[i])) {
      // NOLINTNEXTLINE
      RCLCPP_WARN(get_node()->get_logger(), std::format("Failed to set command for DOF {}", dofs_[i]).c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = time;
    for (const auto && [i, state] : std::views::enumerate(rt_controller_state_pub_->msg_.dof_states)) {
      const auto out = command_interfaces_[i].get_optional();
      state.reference = reference_interfaces_[i];
      state.feedback = system_state_values_[i];
      state.error = error_values[i];
      state.time_step = period.seconds();
      state.output = out.value_or(std::numeric_limits<double>::quiet_NaN());
    }
    rt_controller_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace velocity_controllers

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::IntegralSlidingModeController,
  controller_interface::ChainableControllerInterface)
