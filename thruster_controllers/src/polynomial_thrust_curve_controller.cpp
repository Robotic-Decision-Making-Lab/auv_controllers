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

#include "thruster_controllers/polynomial_thrust_curve_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace thruster_controllers
{

namespace
{

[[nodiscard]] int calculate_pwm_from_thrust_curve(double force, const std::vector<double> & coefficients)
{
  double pwm = 0.0;
  for (size_t i = 0; i < coefficients.size(); ++i) {
    pwm += coefficients[i] * std::pow(force, i);
  }
  return static_cast<int>(std::round(pwm));
}

}  // namespace

controller_interface::CallbackReturn PolynomialThrustCurveController::on_init()
{
  try {
    param_listener_ = std::make_shared<polynomial_thrust_curve_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    fprintf(stderr, "An exception occurred while initializing the controller: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void PolynomialThrustCurveController::update_parameters()  // NOLINT
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn PolynomialThrustCurveController::configure_parameters()
{
  update_parameters();
  thruster_name_ = params_.thruster;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PolynomialThrustCurveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  const auto reference_msg = std::make_shared<std_msgs::msg::Float64>();
  reference_.writeFromNonRT(reference_msg);

  command_interfaces_.reserve(1);

  reference_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<std_msgs::msg::Float64> msg) { reference_.writeFromNonRT(msg); });  // NOLINT

  controller_state_pub_ =
    get_node()->create_publisher<control_msgs::msg::SingleDOFStateStamped>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::SingleDOFStateStamped>>(
      controller_state_pub_);

  rt_controller_state_pub_->lock();
  rt_controller_state_pub_->msg_.dof_state.name = thruster_name_;
  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PolynomialThrustCurveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  (*reference_.readFromNonRT())->data = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PolynomialThrustCurveController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PolynomialThrustCurveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PolynomialThrustCurveController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::InterfaceConfiguration PolynomialThrustCurveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_configuration;
  command_interfaces_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_configuration.names.reserve(1);
  command_interfaces_configuration.names.emplace_back(thruster_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

  return command_interfaces_configuration;
}

controller_interface::InterfaceConfiguration PolynomialThrustCurveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type = controller_interface::interface_configuration_type::NONE;
  return state_interface_configuration;
}

std::vector<hardware_interface::CommandInterface> PolynomialThrustCurveController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  reference_interfaces.emplace_back(
    get_node()->get_name(), thruster_name_ + "/" + hardware_interface::HW_IF_VELOCITY,
    &reference_interfaces_[0]);  // NOLINT

  return reference_interfaces;
}

controller_interface::return_type PolynomialThrustCurveController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto * current_reference = reference_.readFromNonRT();
  reference_interfaces_[0] = (*current_reference)->data;
  (*current_reference)->data = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::return_type::OK;
}

controller_interface::return_type PolynomialThrustCurveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // If the reference is NaN, just apply the NaN to the output
  if (std::isnan(reference_interfaces_[0])) {
    command_interfaces_[0].set_value(reference_interfaces_[0]);
  } else {
    double clamped_reference = std::clamp(reference_interfaces_[0], params_.min_thrust, params_.max_thrust);
    command_interfaces_[0].set_value(
      calculate_pwm_from_thrust_curve(clamped_reference, params_.thrust_curve_coefficients));
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = time;
    rt_controller_state_pub_->msg_.dof_state.reference = reference_interfaces_[0];
    rt_controller_state_pub_->msg_.dof_state.time_step = period.seconds();
    rt_controller_state_pub_->msg_.dof_state.output = command_interfaces_[0].get_value();
    rt_controller_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace thruster_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  thruster_controllers::PolynomialThrustCurveController, controller_interface::ChainableControllerInterface)
