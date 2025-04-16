// Copyright 2025, Evan Palmer
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

#include "thruster_controllers/rotation_rate_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ranges>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace thruster_controllers
{

namespace
{

[[nodiscard]] auto calculate_angular_velocity_from_thrust(
  double thrust,
  double fluid_density,
  double propeller_diameter,
  double thrust_coefficient) -> double
{
  const double ang_vel = sqrt(abs(thrust / (fluid_density * thrust_coefficient * pow(propeller_diameter, 4))));
  return thrust < 0 ? -ang_vel : ang_vel;
}

}  // namespace

auto RotationRateController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<rotation_rate_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

auto RotationRateController::update_parameters() -> void  // NOLINT
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto RotationRateController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();
  thruster_name_ = params_.thruster;
  return controller_interface::CallbackReturn::SUCCESS;
}

auto RotationRateController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();
  reference_.writeFromNonRT(std_msgs::msg::Float64());
  command_interfaces_.reserve(1);

  reference_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<std_msgs::msg::Float64> msg) {  // NOLINT
      reference_.writeFromNonRT(*msg);
    });

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

auto RotationRateController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  reference_.readFromNonRT()->data = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto RotationRateController::command_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interface_config.names.reserve(1);
  command_interface_config.names.emplace_back(std::format("{}/{}", thruster_name_, hardware_interface::HW_IF_VELOCITY));
  return command_interface_config;
}

auto RotationRateController::state_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::NONE;
  return state_interface_config;
}

auto RotationRateController::on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());

  interfaces.emplace_back(
    get_node()->get_name(),
    std::format("{}/{}", thruster_name_, hardware_interface::HW_IF_EFFORT),
    &reference_interfaces_[0]);  // NOLINT(readability-container-data-pointer)

  return interfaces;
}

auto RotationRateController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();
  reference_interfaces_[0] = current_reference->data;
  current_reference->data = std::numeric_limits<double>::quiet_NaN();
  return controller_interface::return_type::OK;
}

auto RotationRateController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  double reference = reference_interfaces_[0];
  if (std::isnan(reference)) {
    if (!command_interfaces_[0].set_value(0.0)) {
      RCLCPP_WARN(  // NOLINT
        get_node()->get_logger(),
        std::format("Failed to set command for thruster {}", thruster_name_).c_str());
    }
  } else {
    reference = std::clamp(reference, params_.min_thrust, params_.max_thrust);
    double ang_vel = calculate_angular_velocity_from_thrust(
      reference, params_.fluid_density, params_.propeller_diameter, params_.thrust_coefficient);
    ang_vel = ang_vel > params_.min_deadband && ang_vel < params_.max_deadband ? 0.0 : ang_vel;

    if (!command_interfaces_[0].set_value(ang_vel)) {
      RCLCPP_WARN(  // NOLINT
        get_node()->get_logger(),
        std::format("Failed to set command for thruster {}", thruster_name_).c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    const auto out = command_interfaces_[0].get_optional();
    rt_controller_state_pub_->msg_.header.stamp = time;
    rt_controller_state_pub_->msg_.dof_state.reference = reference_interfaces_[0];
    rt_controller_state_pub_->msg_.dof_state.time_step = period.seconds();
    rt_controller_state_pub_->msg_.dof_state.output = out.value_or(std::numeric_limits<double>::quiet_NaN());
    rt_controller_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace thruster_controllers

PLUGINLIB_EXPORT_CLASS(thruster_controllers::RotationRateController, controller_interface::ChainableControllerInterface)
