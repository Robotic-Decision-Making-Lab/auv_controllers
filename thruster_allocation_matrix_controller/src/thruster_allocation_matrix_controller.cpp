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

#include "thruster_allocation_matrix_controller/thruster_allocation_matrix_controller.hpp"

#include <Eigen/Dense>
#include <string>

namespace thruster_allocation_matrix_controller
{

ThrusterAllocationMatrixController::CallbackReturn ThrusterAllocationMatrixController::on_init()
{
  // Pulls parameters and updates them - Ever
  try {
    param_listener_ = std::make_shared<thruster_allocation_matrix_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e) {
    fprintf(stderr, "An exception occurred while initializing the controller: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void ThrusterAllocationMatrixController::update_parameters()
{
  // Need this
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
  return;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::configure_parameters()
{
  update_parameters();

  // Update the thruster allocation matrix
  tcm_ = vectorToEigen(params_.tcm, 6, params_.tcm.size() / 6);

  // tcm_ = Eigen::VectorXd(params_.tcm.data()).asDiagonal().toDenseMatrix();

  // Should not need this stuff for our code -Ever

  // // Update the controller gains used by the controller
  // sliding_gain_ = params_.gains.rho;
  // boundary_thickness_ = params_.gains.lambda;
  // proportional_gain_ = Eigen::Vector6d(params_.gains.Kp.data()).asDiagonal().toDenseMatrix();

  // // Update the hydrodynamic parameters used by the controller
  // Eigen::Vector3d moments_of_inertia(params_.hydrodynamics.moments_of_inertia.data());
  // Eigen::Vector6d added_mass(params_.hydrodynamics.added_mass.data());
  // Eigen::Vector6d linear_damping(params_.hydrodynamics.linear_damping.data());
  // Eigen::Vector6d quadratic_damping(params_.hydrodynamics.quadratic_damping.data());
  // Eigen::Vector3d center_of_buoyancy(params_.hydrodynamics.center_of_buoyancy.data());
  // Eigen::Vector3d center_of_gravity(params_.hydrodynamics.center_of_gravity.data());

  // // Don't move the inertial parameters because we use them twice
  // inertia_ = std::make_unique<hydrodynamics::Inertia>(params_.hydrodynamics.mass, moments_of_inertia, added_mass);
  // coriolis_ = std::make_unique<hydrodynamics::Coriolis>(params_.hydrodynamics.mass, moments_of_inertia,
  // added_mass);

  // // Move the damping and restoring forces because we only use them once
  // damping_ = std::make_unique<hydrodynamics::Damping>(std::move(linear_damping), std::move(quadratic_damping));
  // restoring_forces_ = std::make_unique<hydrodynamics::RestoringForces>(
  //   params_.hydrodynamics.weight, params_.hydrodynamics.buoyancy, std::move(center_of_buoyancy),
  //   std::move(center_of_gravity));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ThrusterAllocationMatrixController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ThrusterAllocationMatrixController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

// controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_cleanup(
//   const rclcpp_lifecycle::State & previous_state)
// {
// }

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // NOTE: We may actually need something here - Ever
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // NOTE: We may actually need something here - Ever
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ThrusterAllocationMatrixController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type ThrusterAllocationMatrixController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // I think this is where we actually process the command from previous controller
  // and output the cascaded vector/values
  return controller_interface::return_type::OK;
}

bool ThrusterAllocationMatrixController::on_set_chained_mode(bool chained_mode) { return true; }

std::vector<hardware_interface::CommandInterface> ThrusterAllocationMatrixController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  // TODO: put stuff here
  return reference_interfaces;
}

void ThrusterAllocationMatrixController::reference_state_callback(
  std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg)
{
  return;
}

}  // namespace thruster_allocation_matrix_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  thruster_allocation_matrix_controller::ThrusterAllocationMatrixController,
  controller_interface::ChainableControllerInterface)
