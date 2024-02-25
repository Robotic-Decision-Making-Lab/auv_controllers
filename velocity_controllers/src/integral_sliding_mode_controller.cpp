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
#include <string>

namespace velocity_controllers
{

IntegralSlidingModeController::CallbackReturn IntegralSlidingModeController::on_init()
{
  try {
    param_listener_ = std::make_shared<integral_sliding_mode_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e) {
    fprintf(stderr, "An exception occurred while initializing the controller: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

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

controller_interface::InterfaceConfiguration IntegralSlidingModeController::command_interface_configuration() const {}

controller_interface::InterfaceConfiguration IntegralSlidingModeController::state_interface_configuration() const {}

controller_interface::CallbackReturn IntegralSlidingModeController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
}

controller_interface::CallbackReturn IntegralSlidingModeController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
}

controller_interface::return_type IntegralSlidingModeController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

controller_interface::return_type IntegralSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

bool IntegralSlidingModeController::on_set_chained_mode(bool chained_mode) {}

std::vector<hardware_interface::CommandInterface> IntegralSlidingModeController::on_export_reference_interfaces() {}

void IntegralSlidingModeController::reference_state_callback(std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg) {}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::IntegralSlidingModeController, controller_interface::ChainableControllerInterface)
