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
#include <sstream>
#include <stdexcept>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace velocity_controllers
{

namespace
{

void reset_command_msg(
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> & msg, const std::vector<std::string> & dof_names)
{
  // Resize to the correct number of DoFs
  msg->dof_names = dof_names;
  msg->values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->values_dot.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void sort_command_msg_by_names(
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> & msg, const std::vector<std::string> & dof_names,
  const rclcpp::Logger & logger)
{
  const size_t dof = dof_names.size();

  // Make a copy of the message so that we can sort the original in-place
  auto unsorted_msg = std::make_shared<control_msgs::msg::MultiDOFCommand>(*msg);
  reset_command_msg(msg, dof_names);

  if (unsorted_msg->values_dot.size() != unsorted_msg->values.size()) {
    RCLCPP_DEBUG(  // NOLINT
      logger,
      "Received a command message with mismatched value sizes (%zu values and %zu values_dot). Assuming that "
      "values_dot is not defined.",
      unsorted_msg->values.size(), unsorted_msg->values_dot.size());

    unsorted_msg->values_dot.assign(unsorted_msg->values.size(), std::numeric_limits<double>::quiet_NaN());
  }

  if (unsorted_msg->dof_names.empty() && unsorted_msg->values.size() == dof) {
    RCLCPP_WARN(  // NOLINT
      logger,
      "Received a command message without any DoF names. Assuming that the values are defined in the same order as "
      "the desired DoF names.");

    msg->values = unsorted_msg->values;
    msg->values_dot = unsorted_msg->values_dot;
  } else if (unsorted_msg->dof_names.size() == dof && unsorted_msg->values.size() == dof) {
    for (size_t i = 0; i < dof; ++i) {
      auto it = std::find(unsorted_msg->dof_names.begin(), unsorted_msg->dof_names.end(), dof_names[i]);

      if (it == unsorted_msg->dof_names.end()) {
        throw std::invalid_argument(
          "Received a command message with a DoF name that does not match the controller's DoF names.");
      }

      const size_t index = std::distance(unsorted_msg->dof_names.begin(), it);
      msg->values[i] = unsorted_msg->values[index];
      msg->values_dot[i] = unsorted_msg->values_dot[index];
    }
  } else {
    std::stringstream ss;
    ss << "Reference message had " << unsorted_msg->dof_names.size() << " DoF names and " << unsorted_msg->values.size()
       << " values; expected " << dof << ".";
    throw std::invalid_argument(ss.str());
  }
}

Eigen::Vector6d create_six_dof_eigen_from_named_vector(
  const std::vector<std::string> & dof_names, const std::array<std::string, 6> & six_dof_names,
  const std::vector<double> & values)
{
  if (dof_names.size() != values.size()) {
    throw std::invalid_argument("The DoF names and values must have the same size.");
  }

  Eigen::Vector6d vec = Eigen::Vector6d::Zero();

  for (size_t i = 0; i < six_dof_names.size(); ++i) {
    auto it = std::find(dof_names.begin(), dof_names.end(), six_dof_names[i]);

    if (it == dof_names.end()) {
      vec[i] = std::numeric_limits<double>::quiet_NaN();
    } else {
      vec[i] = values[std::distance(dof_names.begin(), it)];
    }
  }

  return vec;
}

}  // namespace

IntegralSlidingModeController::CallbackReturn IntegralSlidingModeController::on_init()
{
  try {
    param_listener_ = std::make_shared<integral_sliding_mode_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    fprintf(stderr, "An exception occurred while initializing the controller: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IntegralSlidingModeController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_configuration;

  // The ISMC is an effort controller, so we only need to export the effort interface
  for (const auto & name : dof_names_) {
    command_interface_configuration.names.emplace_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return command_interface_configuration;
}

controller_interface::InterfaceConfiguration IntegralSlidingModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_configuration;

  if (params_.use_external_measured_states) {
    state_interface_configuration.type = controller_interface::interface_configuration_type::NONE;
  } else {
    state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // The ISMC only requires velocity state information
    for (const auto & name : dof_names_) {
      state_interface_configuration.names.emplace_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
  }

  return state_interface_configuration;
}

std::vector<hardware_interface::CommandInterface> IntegralSlidingModeController::on_export_reference_interfaces()
{
  // This controller has velocity and acceleration reference interfaces
  const std::array<std::string, 2> reference_interfaces_types = {
    hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION};

  reference_interfaces_.resize(2 * dof_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  for (const auto & reference_type : reference_interfaces_types) {
    for (size_t i = 0; i < dof_; ++i) {
      reference_interfaces.emplace_back(
        get_node()->get_name(), dof_names_[i] + "/" + reference_type, &reference_interfaces_[i]);
    }
  }

  return reference_interfaces;
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void IntegralSlidingModeController::update_parameters()
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn IntegralSlidingModeController::configure_parameters()
{
  update_parameters();

  // These are just used to improve readability
  dof_names_ = params_.dof_names;
  dof_ = dof_names_.size();

  // Update the controller gains used by the controller
  sliding_gain_ = params_.gains.rho;
  boundary_thickness_ = params_.gains.lambda;
  proportional_gain_ = Eigen::Vector6d(params_.gains.Kp.data()).asDiagonal().toDenseMatrix();

  // Update the hydrodynamic parameters used by the controller
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
    params_.hydrodynamics.weight, params_.hydrodynamics.buoyancy, std::move(center_of_buoyancy),
    std::move(center_of_gravity));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Initialize the reference and state realtime messages
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> reference_msg =
    std::make_shared<control_msgs::msg::MultiDOFCommand>();
  reset_command_msg(reference_msg, dof_names_);
  reference_.writeFromNonRT(reference_msg);

  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> system_state_msg =
    std::make_shared<control_msgs::msg::MultiDOFCommand>();
  reset_command_msg(system_state_msg, dof_names_);
  system_state_.writeFromNonRT(system_state_msg);

  // Reset the measured state values
  system_state_values_.resize(dof_, std::numeric_limits<double>::quiet_NaN());

  // Subscribe to the reference topic
  reference_sub_ = get_node()->create_subscription<control_msgs::msg::MultiDOFCommand>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg) { reference_callback(msg); });  // NOLINT

  // Subscribe to the system rotation topic
  system_rotation_sub_ = get_node()->create_subscription<geometry_msgs::msg::Quaternion>(
    "~/system_rotation", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Quaternion> msg) {  // NOLINT
      Eigen::Quaterniond rotation;
      tf2::fromMsg(*msg, rotation);
      system_rotation_.writeFromNonRT(rotation);
    });

  // If we aren't reading from the state interfaces, subscribe to the system state topic
  if (params_.use_external_measured_states) {
    system_state_sub_ = get_node()->create_subscription<control_msgs::msg::MultiDOFCommand>(
      "~/system_state", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg) { state_callback(msg); });  // NOLINT
  }

  // Setup the controller state publisher
  controller_state_pub_ =
    get_node()->create_publisher<control_msgs::msg::MultiDOFStateStamped>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>>(controller_state_pub_);

  // Initialize the controller state message in the realtime publisher
  rt_controller_state_pub_->lock();
  rt_controller_state_pub_->msg_.dof_states.resize(dof_);
  for (size_t i = 0; i < dof_; ++i) {
    rt_controller_state_pub_->msg_.dof_states[i].name = dof_names_[i];
  }
  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

void IntegralSlidingModeController::reference_callback(
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg)  // NOLINT
{
  try {
    sort_command_msg_by_names(msg, dof_names_, get_node()->get_logger());
    reference_.writeFromNonRT(msg);
  }
  catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received an invalid reference message: %s", e.what());  // NOLINT
    return;
  }
}

void IntegralSlidingModeController::state_callback(
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg)  // NOLINT
{
  try {
    sort_command_msg_by_names(msg, dof_names_, get_node()->get_logger());
    system_state_.writeFromNonRT(msg);
  }
  catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received an invalid state message: %s", e.what());  // NOLINT
    return;
  }

  // Update the current system state values
  update_system_state_values();
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Indicate that the next state update will be the first: this is used to set the error initial conditions
  first_update_ = true;

  // Clear the error terms
  total_velocity_error_ = Eigen::Vector6d::Zero();
  initial_velocity_error_ = Eigen::Vector6d::Zero();

  // Reset the rotation
  system_rotation_.writeFromNonRT(Eigen::Quaterniond::Identity());

  // Reset the controller state
  reset_command_msg(*(reference_.readFromRT()), dof_names_);
  reset_command_msg(*(system_state_.readFromRT()), dof_names_);

  // Reset the reference and state interfaces
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool IntegralSlidingModeController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::return_type IntegralSlidingModeController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto * current_reference = reference_.readFromRT();

  for (size_t i = 0; i < dof_; ++i) {
    // Update the velocity reference
    if (!std::isnan((*current_reference)->values[i])) {
      reference_interfaces_[i] = (*current_reference)->values[i];
      (*current_reference)->values[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // Update the acceleration reference
    if (!std::isnan((*current_reference)->values_dot[i])) {
      reference_interfaces_[i + dof_] = (*current_reference)->values_dot[i];
      (*current_reference)->values_dot[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type IntegralSlidingModeController::update_system_state_values()
{
  if (params_.use_external_measured_states) {
    auto * current_system_state = system_state_.readFromRT();

    for (size_t i = 0; i < dof_; ++i) {
      if (!std::isnan((*current_system_state)->values[i])) {
        system_state_values_[i] = (*current_system_state)->values[i];
        (*current_system_state)->values[i] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  } else {
    for (size_t i = 0; i < system_state_values_.size(); ++i) {
      system_state_values_[i] = state_interfaces_[i].get_value();
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type IntegralSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (params_.enable_parameter_update_without_reactivation) {
    configure_parameters();
  }

  // Get the reference acceleration as an Eigen vector
  const std::vector<double> reference_acceleration_values(
    reference_interfaces_.begin() + dof_, reference_interfaces_.end());
  const Eigen::Vector6d reference_acceleration =
    create_six_dof_eigen_from_named_vector(dof_names_, six_dof_names_, reference_acceleration_values);

  // Get the velocity state as an Eigen vector
  const Eigen::Vector6d velocity_state =
    create_six_dof_eigen_from_named_vector(dof_names_, six_dof_names_, system_state_values_);

  // Calculate the velocity error
  std::vector<double> velocity_error_values;
  velocity_error_values.reserve(dof_);

  std::transform(
    reference_interfaces_.begin(), reference_interfaces_.begin() + dof_, system_state_values_.begin(),
    std::back_inserter(velocity_error_values), [](double reference, double state) {
      return !std::isnan(reference) && !std::isnan(state) ? reference - state
                                                          : std::numeric_limits<double>::quiet_NaN();
    });
  const Eigen::Vector6d velocity_error =
    create_six_dof_eigen_from_named_vector(dof_names_, six_dof_names_, velocity_error_values);

  if (first_update_) {
    // If this is the first update, set the initial error conditions
    initial_velocity_error_ = velocity_error;
    first_update_ = false;
  } else {
    // Update the total velocity error
    total_velocity_error_ += velocity_error * period.seconds();
  }

  // Calculate the computed torque control
  const Eigen::Vector6d tau0 =
    inertia_->getMassMatrix() * (reference_acceleration + proportional_gain_ * velocity_error) +
    coriolis_->calculateCoriolisMatrix(velocity_state) * velocity_state +
    damping_->calculateDampingMatrix(velocity_state) * velocity_state +
    restoring_forces_->calculateRestoringForcesVector(system_rotation_.readFromRT()->toRotationMatrix());

  // Calculate the sliding surface
  Eigen::Vector6d surface =
    velocity_error + proportional_gain_ * total_velocity_error_ - proportional_gain_ * initial_velocity_error_;

  // Apply the sign function to the surface
  // Right now, we use the tanh function to reduce the chattering effect.
  // TODO(evan-palmer): Implement the super twisting algorithm to improve this further
  surface = surface.unaryExpr([this](double x) { return std::tanh(x / boundary_thickness_); });

  // Calculate the disturbance rejection torque
  const Eigen::Vector6d tau1 = -sliding_gain_ * surface;

  // Total control torque
  const Eigen::Vector6d tau = tau0 + tau1;

  // Convert the control torque to a command interface value
  for (size_t i = 0; i < dof_; ++i) {
    auto it = std::find(six_dof_names_.begin(), six_dof_names_.end(), dof_names_[i]);
    const size_t index = std::distance(six_dof_names_.begin(), it);
    command_interfaces_[i].set_value(tau[index]);
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = time;
    for (size_t i = 0; i < dof_; ++i) {
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
  velocity_controllers::IntegralSlidingModeController, controller_interface::ChainableControllerInterface)
