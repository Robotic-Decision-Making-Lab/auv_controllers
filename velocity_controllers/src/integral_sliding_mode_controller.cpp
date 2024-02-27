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
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace velocity_controllers
{

namespace
{

void reset_command_msg(
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> & msg, const std::vector<std::string> & dof_names)
{
  // Allocate memory for the message/clear previous values
  msg->dof_names = dof_names;
  msg->values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->values_dot.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void sort_command_msg_by_names(
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> & msg, const std::vector<std::string> & dof_names,
  const rclcpp::Logger & logger)
{
  const size_t dof = dof_names.size();

  auto unsorted_msg = std::make_shared<control_msgs::msg::MultiDOFCommand>(*msg);
  reset_command_msg(msg, dof_names);

  if (unsorted_msg->dof_names.empty() && unsorted_msg->values.size() == dof) {
    RCLCPP_WARN(  // NOLINT
      logger,
      "Received a command message without any DoF names. Assuming that the values are defined in the same order as "
      "the desired DoF names.");

    msg->values = unsorted_msg->values;
    msg->values_dot = unsorted_msg->values_dot;
  } else if (unsorted_msg->dof_names.size() == dof && unsorted_msg->values.size() == dof) {
    if (unsorted_msg->values_dot.size() != unsorted_msg->values.size()) {
      RCLCPP_WARN(  // NOLINT
        logger,
        "Received a command message with mismatched value sizes (%zu values and %zu values_dot). Assuming that "
        "values_dot is not defined.",
        unsorted_msg->values.size(), unsorted_msg->values_dot.size());

      unsorted_msg->values_dot.resize(unsorted_msg->values.size(), std::numeric_limits<double>::quiet_NaN());
    }

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

    // The ISMC requires velocity and acceleration state information
    const std::array<std::string, 2> state_interfaces = {
      hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION};

    for (const auto & state_interface : state_interfaces) {
      for (const auto & name : dof_names_) {
        state_interface_configuration.names.emplace_back(name + "/" + state_interface);
      }
    }
  }

  return state_interface_configuration;
}

std::vector<hardware_interface::CommandInterface> IntegralSlidingModeController::on_export_reference_interfaces()
{
  // This controller has a velocity and acceleration reference interface
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
  Eigen::Vector3d moments_of_inertia(params_.hydrodynamics.moments_of_inertia.data());
  Eigen::Vector6d added_mass(params_.hydrodynamics.added_mass.data());
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
  auto success = configure_parameters();
  if (success != controller_interface::CallbackReturn::SUCCESS) {
    return success;
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

  // Reset the measured state values; there are 2 state values for each DOF
  system_state_values_.resize(2 * dof_, std::numeric_limits<double>::quiet_NaN());

  // Subscribe to the reference topic
  reference_sub_ = get_node()->create_subscription<control_msgs::msg::MultiDOFCommand>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg) { reference_callback(msg); });

  // If we aren't reading from the state interfaces, subscribe to the system state topic
  // TODO(evan-palmer): Move this to a different cb to sort the values
  if (params_.use_external_measured_states) {
    system_state_sub_ = get_node()->create_subscription<control_msgs::msg::MultiDOFCommand>(
      "~/system_state", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg) { system_state_.writeFromNonRT(msg); });
  }

  // Setup the controller state publisher
  controller_state_pub_ =
    get_node()->create_publisher<control_msgs::msg::MultiDOFStateStamped>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>>(controller_state_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

void IntegralSlidingModeController::reference_callback(const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg)
{
  try {
    sort_command_msg_by_names(msg, dof_names_, get_node()->get_logger());
    reference_.writeFromNonRT(msg);
  }
  catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received an invalid reference message: %s", e.what());
    return;
  }
}

void IntegralSlidingModeController::state_callback(const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg)
{
  try {
    sort_command_msg_by_names(msg, dof_names_, get_node()->get_logger());
    system_state_.writeFromNonRT(msg);
  }
  catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received an invalid state message: %s", e.what());  // NOLINT
    return;
  }
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Indicate that the next state update will be the first: this is used to set the error initial conditions
  first_update_ = true;

  // Clear the error terms
  total_velocity_error_ = Eigen::Vector6d::Zero();
  initial_velocity_error_ = Eigen::Vector6d::Zero();
  initial_acceleration_error_ = Eigen::Vector6d::Zero();

  // Reset the controller state
  reset_command_msg(*(reference_.readFromRT()), dof_names_);
  reset_command_msg(*(system_state_.readFromRT()), dof_names_);

  // Reset the reference interfaces
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

controller_interface::return_type IntegralSlidingModeController::update_system_state_from_subscribers()
{
  auto * current_system_state = system_state_.readFromRT();

  for (size_t i = 0; i < dof_; ++i) {
    // Update the velocity state
    if (!std::isnan((*current_system_state)->values[i])) {
      system_state_values_[i] = (*current_system_state)->values[i];
      (*current_system_state)->values[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // Update the acceleration state
    if (!std::isnan((*current_system_state)->values_dot[i])) {
      system_state_values_[i + dof_] = (*current_system_state)->values_dot[i];
      (*current_system_state)->values_dot[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type IntegralSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::IntegralSlidingModeController, controller_interface::ChainableControllerInterface)
