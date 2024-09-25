// Copyright 2024, Evan Palmer, Colin Mitchell, Everardo Gonzalez, Rakesh Vivekanandan
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
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace thruster_allocation_matrix_controller
{

namespace
{

auto reset_wrench_msg(geometry_msgs::msg::Wrench * wrench_msg) -> void  // NOLINT
{
  wrench_msg->force.x = std::numeric_limits<double>::quiet_NaN();
  wrench_msg->force.y = std::numeric_limits<double>::quiet_NaN();
  wrench_msg->force.z = std::numeric_limits<double>::quiet_NaN();
  wrench_msg->torque.x = std::numeric_limits<double>::quiet_NaN();
  wrench_msg->torque.y = std::numeric_limits<double>::quiet_NaN();
  wrench_msg->torque.z = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

auto ThrusterAllocationMatrixController::on_init() -> controller_interface::CallbackReturn
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

auto ThrusterAllocationMatrixController::update_parameters() -> void  // NOLINT
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto ThrusterAllocationMatrixController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();

  thruster_names_ = params_.thrusters;
  num_thrusters_ = thruster_names_.size();

  // Make sure that the number prefixes (if provided) provided match the number of thrusters
  if (!params_.reference_controllers.empty() && params_.reference_controllers.size() != num_thrusters_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mismatched number of command interface prefixes and thrusters. Expected %ld, got %ld.",
      num_thrusters_,
      params_.reference_controllers.size());

    return controller_interface::CallbackReturn::ERROR;
  }

  const std::vector<std::vector<double>> vecs = {
    params_.tam.x, params_.tam.y, params_.tam.z, params_.tam.rx, params_.tam.ry, params_.tam.rz};

  // Make sure that all of the rows are the same size
  for (const auto & vec : vecs) {
    const std::size_t vec_size = vec.size();
    if (vec_size != num_thrusters_) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Mismatched TAM row sizes. Expected %ld thrusters, got %ld.",
        num_thrusters_,
        vec_size);

      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Eigen will always convert a dynamic matrix lvalue to match the size of the rvalue
  tam_ = Eigen::MatrixXd::Zero(DOF, num_thrusters_);

  tam_ << Eigen::RowVectorXd::Map(params_.tam.x.data(), num_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.y.data(), num_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.z.data(), num_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.rx.data(), num_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.ry.data(), num_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.rz.data(), num_thrusters_);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ThrusterAllocationMatrixController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  auto ret = configure_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  reference_.writeFromNonRT(geometry_msgs::msg::Wrench());

  command_interfaces_.reserve(num_thrusters_);

  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
    "~/reference",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Wrench> msg) {  // NOLINT
      reference_.writeFromNonRT(*msg);
    });

  controller_state_pub_ = get_node()->create_publisher<auv_control_msgs::msg::MultiActuatorStateStamped>(
    "~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<auv_control_msgs::msg::MultiActuatorStateStamped>>(
      controller_state_pub_);

  rt_controller_state_pub_->lock();
  rt_controller_state_pub_->msg_.output_names = thruster_names_;
  rt_controller_state_pub_->msg_.reference_names.assign(dof_names_.begin(), dof_names_.end());
  rt_controller_state_pub_->msg_.reference.resize(DOF, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->msg_.output.resize(num_thrusters_, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ThrusterAllocationMatrixController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  reset_wrench_msg(reference_.readFromNonRT());
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto ThrusterAllocationMatrixController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto ThrusterAllocationMatrixController::command_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration command_interfaces_configuration;
  command_interfaces_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_configuration.names.reserve(num_thrusters_);

  for (std::size_t i = 0; i < num_thrusters_; ++i) {
    if (params_.reference_controllers.empty()) {
      command_interfaces_configuration.names.emplace_back(thruster_names_[i] + "/" + hardware_interface::HW_IF_EFFORT);
    } else {
      command_interfaces_configuration.names.emplace_back(
        params_.reference_controllers[i] + "/" + thruster_names_[i] + "/" + hardware_interface::HW_IF_EFFORT);
    }
  }

  return command_interfaces_configuration;
}

auto ThrusterAllocationMatrixController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type = controller_interface::interface_configuration_type::NONE;
  return state_interface_configuration;
}

auto ThrusterAllocationMatrixController::on_export_reference_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(DOF, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  for (std::size_t i = 0; i < DOF; ++i) {
    reference_interfaces.emplace_back(
      get_node()->get_name(), dof_names_[i] + "/" + hardware_interface::HW_IF_EFFORT, &reference_interfaces_[i]);
  }

  return reference_interfaces;
}

auto ThrusterAllocationMatrixController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();

  const std::vector<double> wrench = {
    current_reference->force.x,
    current_reference->force.y,
    current_reference->force.z,
    current_reference->torque.x,
    current_reference->torque.y,
    current_reference->torque.z};

  for (std::size_t i = 0; i < wrench.size(); ++i) {
    if (!std::isnan(wrench[i])) {
      reference_interfaces_[i] = wrench[i];
    }
  }

  reset_wrench_msg(current_reference);

  return controller_interface::return_type::OK;
}

auto ThrusterAllocationMatrixController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) -> controller_interface::return_type
{
  const Eigen::Vector6d reference_wrench(reference_interfaces_.data());
  const Eigen::VectorXd thrust(tam_.completeOrthogonalDecomposition().pseudoInverse() * reference_wrench);

  for (std::size_t i = 0; i < num_thrusters_; i++) {
    command_interfaces_[i].set_value(thrust[i]);
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = time;
    rt_controller_state_pub_->msg_.time_step = period.seconds();
    rt_controller_state_pub_->msg_.reference = reference_interfaces_;

    for (std::size_t i = 0; i < num_thrusters_; i++) {
      rt_controller_state_pub_->msg_.output[i] = command_interfaces_[i].get_value();
    }

    rt_controller_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace thruster_allocation_matrix_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  thruster_allocation_matrix_controller::ThrusterAllocationMatrixController,
  controller_interface::ChainableControllerInterface)
