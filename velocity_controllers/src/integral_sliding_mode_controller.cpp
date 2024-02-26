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

void reset_controller_command_msg(
  const std::shared_ptr<control_msgs::msg::MultiDOFCommand> & msg, const std::vector<std::string> & dof_names)
{
  // Allocate memory for the message/clear previous values
  msg->dof_names = dof_names;
  msg->values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->values_dot.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void read_named_data_into_eigen(
  const std::vector<std::string> & names, const std::vector<double> & data, Eigen::Vector6d & vec)
{
  /* data */
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
    command_interface_configuration.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
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
    std::array<std::string, 2> state_interfaces = {
      hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION};

    for (const auto & state_interface : state_interfaces) {
      for (const auto & name : dof_names_) {
        state_interface_configuration.names.push_back(name + "/" + state_interface);
      }
    }
  }

  return state_interface_configuration;
}

std::vector<hardware_interface::CommandInterface> IntegralSlidingModeController::on_export_reference_interfaces()
{
  // This controller has a velocity and acceleration reference interface
  std::array<std::string, 2> reference_interfaces_types = {
    hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION};

  reference_interfaces_.resize(2 * dof_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  for (const auto & reference_type : reference_interfaces_types) {
    for (size_t i = 0; i < dof_; ++i) {
      reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), dof_names_[i] + "/" + reference_type, &reference_interfaces_[i]));
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
  std::shared_ptr<control_msgs::msg::MultiDOFCommand> reference_msg =
    std::make_shared<control_msgs::msg::MultiDOFCommand>();
  reset_controller_command_msg(reference_msg, dof_names_);
  reference_.writeFromNonRT(reference_msg);

  std::shared_ptr<control_msgs::msg::MultiDOFCommand> system_state_msg =
    std::make_shared<control_msgs::msg::MultiDOFCommand>();
  reset_controller_command_msg(system_state_msg, dof_names_);
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
  if (msg->dof_names.empty() && msg->values.size() == dof_) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received a reference message without any DOF names. Assuming that the values are defined in the same order as "
      "the controller's DOF names.");

    // Assume that the values are in the same order as the controller's DOF names and save them
    auto reference = *msg;
    reference.dof_names = dof_names_;
    reference_.writeFromNonRT(std::make_shared<control_msgs::msg::MultiDOFCommand>(reference));
  } else if (msg->dof_names.size() == dof_ && msg->values.size() == dof_) {
    auto reference = *msg;
    reference.dof_names = dof_names_;

    if (msg->values_dot.size() != msg->values.size()) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Received a reference message with %zu values and %zu values_dot. Assuming that the values_dot are not "
        "defined.",
        msg->values.size(), msg->values_dot.size());
      reference.values_dot.resize(msg->values.size(), 0.0);
    }

    // Sort the values according to the provided DOF names; we want the values to be in the same order as the controller
    for (size_t i = 0; i < dof_; ++i) {
      auto it = std::find(msg->dof_names.begin(), msg->dof_names.end(), dof_names_[i]);

      if (it == msg->dof_names.end()) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Received a reference message with a DOF name that does not match the controller's DOF names. Ignoring the "
          "reference message.");
        return;
      }

      size_t index = std::distance(msg->dof_names.begin(), it);
      reference.values[i] = msg->values[index];
      reference.values_dot[i] = msg->values_dot[index];
    }

    reference_.writeFromNonRT(std::make_shared<control_msgs::msg::MultiDOFCommand>(reference));
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received an invalid reference message. Reference message had %zu DOF names and %zu values; expected %zu.",
      msg->dof_names.size(), msg->values.size(), dof_);
  }
}

void IntegralSlidingModeController::state_callback(const std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg)
{
  /* data */
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
  reset_controller_command_msg(*(reference_.readFromRT()), dof_names_);
  reset_controller_command_msg(*(system_state_.readFromRT()), dof_names_);

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
  auto current_reference = reference_.readFromRT();

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

controller_interface::return_type IntegralSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::IntegralSlidingModeController, controller_interface::ChainableControllerInterface)
