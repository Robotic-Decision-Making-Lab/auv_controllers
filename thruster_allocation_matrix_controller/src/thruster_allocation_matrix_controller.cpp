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
#include <format>
#include <ranges>
#include <stdexcept>

#include "controller_common/common.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace thruster_allocation_matrix_controller
{

<<<<<<< HEAD
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

=======
>>>>>>> origin/main
auto ThrusterAllocationMatrixController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<thruster_allocation_matrix_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
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

  dofs_ = params_.joints;
  n_dofs_ = dofs_.size();

  thruster_names_ = params_.thrusters;
  n_thrusters_ = thruster_names_.size();

  // Make sure that the number prefixes (if provided) provided match the number of thrusters
  if (!params_.reference_controllers.empty() && params_.reference_controllers.size() != n_thrusters_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      std::format(
        "Mismatched number of command interface prefixes and thrusters. Expected {}, got {}.",
        n_thrusters_,
        params_.reference_controllers.size())
        .c_str());

    return controller_interface::CallbackReturn::ERROR;
  }

  const std::vector<std::vector<double>> vecs = {
    params_.tam.x, params_.tam.y, params_.tam.z, params_.tam.rx, params_.tam.ry, params_.tam.rz};

  // Make sure that all of the rows are the same size
  if (std::ranges::any_of(vecs, [this](const auto & vec) { return vec.size() != n_thrusters_; })) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      std::format("Mismatched TAM row sizes. Expected {}, got {}.", n_thrusters_, vecs[0].size()).c_str());

    return controller_interface::CallbackReturn::ERROR;
  }

  tam_ = Eigen::MatrixXd::Zero(n_dofs_, n_thrusters_);
  tam_ << Eigen::RowVectorXd::Map(params_.tam.x.data(), n_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.y.data(), n_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.z.data(), n_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.rx.data(), n_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.ry.data(), n_thrusters_),
    Eigen::RowVectorXd::Map(params_.tam.rz.data(), n_thrusters_);

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
<<<<<<< HEAD

  command_interfaces_.reserve(num_thrusters_);
=======
  command_interfaces_.reserve(n_thrusters_);
>>>>>>> origin/main

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
  rt_controller_state_pub_->msg_.reference_names.assign(dofs_.begin(), dofs_.end());
  rt_controller_state_pub_->msg_.reference.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->msg_.output.resize(n_thrusters_, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ThrusterAllocationMatrixController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
<<<<<<< HEAD
  reset_wrench_msg(reference_.readFromNonRT());
=======
  common::messages::reset_message(reference_.readFromNonRT());
>>>>>>> origin/main
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto ThrusterAllocationMatrixController::command_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_thrusters_);

  for (std::size_t i = 0; i < n_thrusters_; ++i) {
    config.names.emplace_back(
      params_.reference_controllers.empty()
        ? std::format("{}/{}", thruster_names_[i], hardware_interface::HW_IF_EFFORT)
        : std::format(
            "{}/{}/{}", params_.reference_controllers[i], thruster_names_[i], hardware_interface::HW_IF_EFFORT));
  }

  return config;
}

auto ThrusterAllocationMatrixController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type = controller_interface::interface_configuration_type::NONE;
  return state_interface_configuration;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto ThrusterAllocationMatrixController::on_export_reference_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());

  for (const auto & dof : dofs_) {
    interfaces.emplace_back(
      get_node()->get_name(),
      std::format("{}/{}", dof, hardware_interface::HW_IF_EFFORT),
      &reference_interfaces_[interfaces.size()]);
  }

  return interfaces;
}

auto ThrusterAllocationMatrixController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();
<<<<<<< HEAD

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

=======
  const std::vector<double> wrench = common::messages::to_vector(*current_reference);
  for (auto && [interface, ref] : std::views::zip(reference_interfaces_, wrench)) {
    if (!std::isnan(ref)) {
      interface = ref;
    }
  }
  common::messages::reset_message(current_reference);
>>>>>>> origin/main
  return controller_interface::return_type::OK;
}

auto ThrusterAllocationMatrixController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) -> controller_interface::return_type
{
  const Eigen::Vector6d reference_wrench(reference_interfaces_.data());
  const Eigen::VectorXd thrust(tam_.completeOrthogonalDecomposition().pseudoInverse() * reference_wrench);

  for (auto && [interface, value] : std::views::zip(command_interfaces_, thrust)) {
    if (!interface.set_value(value)) {
      RCLCPP_INFO(
        get_node()->get_logger(), std::format("Failed to set command for thruster {}", interface.get_name()).c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = time;
    rt_controller_state_pub_->msg_.time_step = period.seconds();
    rt_controller_state_pub_->msg_.reference = reference_interfaces_;

    for (std::size_t i = 0; i < n_thrusters_; i++) {
      const auto out = command_interfaces_[i].get_optional();
      rt_controller_state_pub_->msg_.output[i] = out.value_or(std::numeric_limits<double>::quiet_NaN());
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
