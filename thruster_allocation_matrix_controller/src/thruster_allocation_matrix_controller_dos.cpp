// Copyright 2024, Colin Mitchell, Everardo Gonzalez, Rakesh Vivekanandan
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

namespace thruster_allocation_matrix_controller
{

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_init()
{
  try {
    param_listener_ = std::make_shared<thruster_allocation_matrix_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    fprintf(stderr, "An exception occurred while initializing the controller: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void ThrusterAllocationMatrixController::update_parameters()
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::configure_parameters()
{
  update_parameters();

  // These are just used to improve readability
  dof_names_ = params_.dof_names;
  dof_ = dof_names_.size();

  // Start by assuming that the number of thrusters is the size of the x vector
  num_thrusters_ = params_.tam.x.size();

  std::vector<std::vector<double>> tam_vecs = {params_.tam.x,  params_.tam.y,  params_.tam.z,
                                               params_.tam.rx, params_.tam.ry, params_.tam.rz};

  // Make sure that all of the rows are the same size
  for (const auto & vec : tam_vecs) {
    size_t vec_size = vec.size();
    if (vec_size != num_thrusters_) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Mismatched TAM vector sizes. Expected %d, got %d.", num_thrusters_, vec_size);

      return controller_interface::CallbackReturn::ERROR;
    }
  }

  tam_ = std::make_unique<Eigen::MatrixXd>(dof_, num_thrusters_);

  tam_ << tam_x.transpose(), tam_y.transpose(), tam_z.transpose(), tam_rx.transpose(), tam_ry.transpose(),
    tam_rz.transpose();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
}

bool on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::InterfaceConfiguration ThrusterAllocationMatrixController::command_interface_configuration() const
{
}

controller_interface::InterfaceConfiguration ThrusterAllocationMatrixController::state_interface_configuration() const
{
}

std::vector<hardware_interface::CommandInterface> ThrusterAllocationMatrixController::on_export_reference_interfaces()
{
}

controller_interface::return_type ThrusterAllocationMatrixController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

controller_interface::return_type ThrusterAllocationMatrixController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

}  // namespace thruster_allocation_matrix_controller
