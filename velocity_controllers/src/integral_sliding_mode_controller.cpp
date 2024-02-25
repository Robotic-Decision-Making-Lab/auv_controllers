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

namespace velocity_controllers
{

IntegralSlidingModeController::CallbackReturn IntegralSlidingModeController::on_init() {}

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

void IntegralSlidingModeController::update_parameters() {}

controller_interface::CallbackReturn IntegralSlidingModeController::configure_parameters() {}

std::vector<hardware_interface::CommandInterface> IntegralSlidingModeController::on_export_reference_interfaces() {}

void IntegralSlidingModeController::reference_state_callback(std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg) {}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::IntegralSlidingModeController, controller_interface::ChainableControllerInterface)
