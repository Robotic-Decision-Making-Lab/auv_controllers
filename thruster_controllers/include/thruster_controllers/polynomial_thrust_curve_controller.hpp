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
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include "thruster_controllers/visibility_control.h"

// auto-generated by generate_parameter_library
#include "polynomial_thrust_curve_controller_parameters.hpp"

namespace thruster_controllers
{

/**
 * @brief Controller used to convert thruster forces into PWM signals using a polynomial thrust curve.
 */
class PolynomialThrustCurveController : public controller_interface::ChainableControllerInterface
{
public:
  THRUSTER_CONTROLLERS_PUBLIC
  PolynomialThrustCurveController() = default;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_CONTROLLERS_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  THRUSTER_CONTROLLERS_PUBLIC
  bool on_set_chained_mode(bool chained_mode) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void update_parameters();

  controller_interface::CallbackReturn configure_parameters();

  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> reference_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> reference_sub_;

  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::SingleDOFStateStamped>> controller_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::SingleDOFStateStamped>> rt_controller_state_pub_;

  std::shared_ptr<polynomial_thrust_curve_controller::ParamListener> param_listener_;
  polynomial_thrust_curve_controller::Params params_;

  std::string thruster_name_;
};

}  // namespace thruster_controllers
