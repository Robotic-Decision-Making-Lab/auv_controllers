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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/single_dof_state_stamped.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64.hpp"

// auto-generated by generate_parameter_library
#include <thruster_controllers/polynomial_thrust_curve_controller_parameters.hpp>

namespace thruster_controllers
{

class PolynomialThrustCurveController : public controller_interface::ChainableControllerInterface
{
public:
  PolynomialThrustCurveController() = default;

  auto on_init() -> controller_interface::CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

protected:
  auto on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto update_parameters() -> void;

  auto configure_parameters() -> controller_interface::CallbackReturn;

  realtime_tools::RealtimeBuffer<std_msgs::msg::Float64> reference_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> reference_sub_;

  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::SingleDOFStateStamped>> controller_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::SingleDOFStateStamped>> rt_controller_state_pub_;

  std::shared_ptr<polynomial_thrust_curve_controller::ParamListener> param_listener_;
  polynomial_thrust_curve_controller::Params params_;

  std::string thruster_name_;

  rclcpp::Logger logger_{rclcpp::get_logger("polynomial_thrust_curve_controller")};
};

}  // namespace thruster_controllers
