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
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
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

}  // namespace

auto IntegralSlidingModeController::on_init() -> controller_interface::CallbackReturn
{
  try {
    param_listener_ = std::make_shared<integral_sliding_mode_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    fprintf(stderr, "An exception occurred while initializing the controller: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Notify users about this. This can be confusing for folks that expect the controller to work without a reference
  // or state message.
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Reference and state messages are required for operation - commands will not be sent until both are received.");

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::command_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & dof : dof_names_) {
    if (params_.reference_controller.empty()) {
      command_interface_configuration.names.emplace_back(dof + "/" + hardware_interface::HW_IF_EFFORT);
    } else {
      command_interface_configuration.names.emplace_back(
        params_.reference_controller + "/" + dof + "/" + hardware_interface::HW_IF_EFFORT);
    }
  }

  return command_interface_configuration;
}

auto IntegralSlidingModeController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration state_interface_configuration;

  if (params_.use_external_measured_states) {
    state_interface_configuration.type = controller_interface::interface_configuration_type::NONE;
  } else {
    state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & name : dof_names_) {
      state_interface_configuration.names.emplace_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
  }

  return state_interface_configuration;
}

auto IntegralSlidingModeController::on_export_reference_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(DOF, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  for (std::size_t i = 0; i < DOF; ++i) {
    reference_interfaces.emplace_back(
      get_node()->get_name(), dof_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[i]);
  }

  return reference_interfaces;
}

auto IntegralSlidingModeController::on_cleanup(const rclcpp_lifecycle::State & /*previous state*/)
  -> controller_interface::CallbackReturn
{
  return controller_interface::CallbackReturn::SUCCESS;
}

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

  // Update the controller gains
  sliding_gain_ = params_.gains.rho;
  boundary_thickness_ = params_.gains.lambda;
  proportional_gain_ = Eigen::Vector6d(params_.gains.Kp.data()).asDiagonal().toDenseMatrix();

  // Update the TF frames
  vehicle_frame_id_ = params_.tf.base_frame;
  inertial_frame_id_ = params_.tf.odom_frame;

  // Update the hydrodynamic parameters
  // TODO(evan-palmer): Read these from the robot_description instead of a parameters file
  const Eigen::Vector3d moments_of_inertia(params_.hydrodynamics.moments_of_inertia.data());
  const Eigen::Vector6d added_mass(params_.hydrodynamics.added_mass.data());
  Eigen::Vector6d linear_damping(params_.hydrodynamics.linear_damping.data());
  Eigen::Vector6d quadratic_damping(params_.hydrodynamics.quadratic_damping.data());
  Eigen::Vector3d center_of_buoyancy(params_.hydrodynamics.center_of_buoyancy.data());
  Eigen::Vector3d center_of_gravity(params_.hydrodynamics.center_of_gravity.data());

  // Don't move the inertial parameters because we use them twice
  inertia_ = std::make_unique<hydrodynamics::Inertia>(params_.hydrodynamics.mass, moments_of_inertia, added_mass);
  coriolis_ = std::make_unique<hydrodynamics::Coriolis>(params_.hydrodynamics.mass, moments_of_inertia, added_mass);

  // Move the damping and restoring forces because we only use them once
  damping_ = std::make_unique<hydrodynamics::Damping>(std::move(linear_damping), std::move(quadratic_damping));
  restoring_forces_ = std::make_unique<hydrodynamics::RestoringForces>(
    params_.hydrodynamics.weight,
    params_.hydrodynamics.buoyancy,
    std::move(center_of_buoyancy),
    std::move(center_of_gravity));

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  auto ret = configure_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  reference_.writeFromNonRT(geometry_msgs::msg::Twist());
  system_state_.writeFromNonRT(geometry_msgs::msg::Twist());

  command_interfaces_.reserve(DOF);

  system_state_values_.resize(DOF, std::numeric_limits<double>::quiet_NaN());

  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/reference",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {  // NOLINT
      reference_.writeFromNonRT(*msg);
    });  // NOLINT

  // If we aren't reading from the state interfaces, subscribe to the system state topic
  if (params_.use_external_measured_states) {
    system_state_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/system_state",
      rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {  // NOLINT
        system_state_.writeFromNonRT(msg->twist);
      });
  }

  // Configure the TF buffer and listener
  // This will be slower than just reading the current system pose from a subscriber, but it's less overhead for users
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  controller_state_pub_ =
    get_node()->create_publisher<control_msgs::msg::MultiDOFStateStamped>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>>(controller_state_pub_);

  rt_controller_state_pub_->lock();
  rt_controller_state_pub_->msg_.dof_states.resize(DOF);
  for (std::size_t i = 0; i < DOF; ++i) {
    rt_controller_state_pub_->msg_.dof_states[i].name = dof_names_[i];
  }
  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  // Indicate that the next state update will be the first: this is used to set the error initial conditions
  first_update_ = true;

  total_velocity_error_ = Eigen::Vector6d::Zero();
  initial_velocity_error_ = Eigen::Vector6d::Zero();

  system_rotation_.writeFromNonRT(Eigen::Quaterniond::Identity());

  reset_twist_msg(reference_.readFromNonRT());
  reset_twist_msg(system_state_.readFromNonRT());

  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  return controller_interface::CallbackReturn::SUCCESS;
}

auto IntegralSlidingModeController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto IntegralSlidingModeController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();

  const std::vector<double> reference = {
    current_reference->linear.x,
    current_reference->linear.y,
    current_reference->linear.z,
    current_reference->angular.x,
    current_reference->angular.y,
    current_reference->angular.z};

  for (std::size_t i = 0; i < reference.size(); ++i) {
    if (!std::isnan(reference[i])) {
      reference_interfaces_[i] = reference[i];
    }
  }

  reset_twist_msg(current_reference);

  return controller_interface::return_type::OK;
}

auto IntegralSlidingModeController::update_system_state_values() -> controller_interface::return_type
{
  if (params_.use_external_measured_states) {
    auto * current_system_state = system_state_.readFromRT();

    const std::vector<double> state = {
      current_system_state->linear.x,
      current_system_state->linear.y,
      current_system_state->linear.z,
      current_system_state->angular.x,
      current_system_state->angular.y,
      current_system_state->angular.z};

    for (std::size_t i = 0; i < state.size(); ++i) {
      if (!std::isnan(state[i])) {
        system_state_values_[i] = state[i];
      }
    }

    reset_twist_msg(current_system_state);
  } else {
    for (std::size_t i = 0; i < system_state_values_.size(); ++i) {
      system_state_values_[i] = state_interfaces_[i].get_value();
    }
  }

  return controller_interface::return_type::OK;
}

auto IntegralSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) -> controller_interface::return_type
{
  if (params_.enable_parameter_update_without_reactivation) {
    configure_parameters();
  }

  update_system_state_values();
  const Eigen::Vector6d velocity_state(system_state_values_.data());

  // Calculate the velocity error
  std::vector<double> velocity_error_values;
  velocity_error_values.reserve(DOF);
  std::transform(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    system_state_values_.begin(),
    std::back_inserter(velocity_error_values),
    [](double reference, double state) {
      return !std::isnan(reference) && !std::isnan(state) ? reference - state
                                                          : std::numeric_limits<double>::quiet_NaN();
    });

  // Filter out NaN values. This will cause issues in the control update
  auto all_nan =
    std ::all_of(velocity_error_values.begin(), velocity_error_values.end(), [&](double i) { return std::isnan(i); });
  if (all_nan) {
    RCLCPP_DEBUG(get_node()->get_logger(), "All velocity error values are NaN. Skipping control update.");
    return controller_interface::return_type::OK;
  }

  const Eigen::Vector6d velocity_error(velocity_error_values.data());

  if (first_update_) {
    // If this is the first update and we have a valid error value, set the initial error conditions
    initial_velocity_error_ = velocity_error;
    first_update_ = false;
  } else {
    // Update the total velocity error
    total_velocity_error_ += velocity_error * period.seconds();
  }

  // Try getting the latest orientation of the vehicle in the inertial frame. If the transform is not available, use the
  // last known orientation.
  try {
    const geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform(inertial_frame_id_, vehicle_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(t.transform.rotation, *system_rotation_.readFromRT());
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Could not transform %s to %s. Using latest available transform. %s",
      inertial_frame_id_.c_str(),
      vehicle_frame_id_.c_str(),
      e.what());
  }

  // Calculate the computed torque control
  // Assume a feed-forward acceleration of 0
  const Eigen::Vector6d tau0 =
    inertia_->mass_matrix * (proportional_gain_ * velocity_error) +
    coriolis_->calculate_coriolis_matrix(velocity_state) * velocity_state +
    damping_->calculate_damping_matrix(velocity_state) * velocity_state +
    restoring_forces_->calculate_restoring_forces_vector(system_rotation_.readFromRT()->toRotationMatrix());

  // Calculate the sliding surface
  Eigen::Vector6d surface =
    velocity_error + proportional_gain_ * total_velocity_error_ - proportional_gain_ * initial_velocity_error_;

  // Apply the sign function to the surface
  // Right now, we use the tanh function to reduce the chattering effect.
  surface = surface.unaryExpr([this](double x) { return std::tanh(x / boundary_thickness_); });

  // Calculate the disturbance rejection torque
  const Eigen::Vector6d tau1 = sliding_gain_ * surface;

  // Total control torque
  const Eigen::Vector6d tau = tau0 + tau1;

  for (std::size_t i = 0; i < DOF; ++i) {
    command_interfaces_[i].set_value(tau[i]);
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = time;
    for (std::size_t i = 0; i < DOF; ++i) {
      rt_controller_state_pub_->msg_.dof_states[i].reference = reference_interfaces_[i];
      rt_controller_state_pub_->msg_.dof_states[i].feedback = system_state_values_[i];
      rt_controller_state_pub_->msg_.dof_states[i].error = velocity_error_values[i];
      rt_controller_state_pub_->msg_.dof_states[i].time_step = period.seconds();
      rt_controller_state_pub_->msg_.dof_states[i].output = command_interfaces_[i].get_value();
    }
    rt_controller_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::IntegralSlidingModeController,
  controller_interface::ChainableControllerInterface)
