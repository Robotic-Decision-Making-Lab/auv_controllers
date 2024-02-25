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

#include <Eigen/Dense>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hydrodynamics/eigen.hpp"
#include "hydrodynamics/hydrodynamics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "velocity_controllers/visibility_control.h"

// auto-generated by generate_parameter_library
// #include "integral_sliding_mode_controller_parameters.hpp"

namespace velocity_controllers
{

/**
 * @brief Integral sliding mode controller (ISMC) for velocity control of an autonomous underwater vehicle.
 */
class IntegralSlidingModeController : public controller_interface::ChainableControllerInterface
{
public:
  VELOCITY_CONTROLLERS_PUBLIC
  IntegralSlidingModeController() = default;

  VELOCITY_CONTROLLERS_PUBLIC controller_interface::CallbackReturn on_init() override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  bool on_set_chained_mode(bool chained_mode) override;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // Keep track of the degree-of-freedom names
  std::vector<std::string> reference_and_state_dof_names_;
  size_t dof_;

  // Reference signal to track
  realtime_tools::RealtimeBuffer<std::shared_ptr<control_msgs::msg::MultiDOFCommand>> reference_;
  rclcpp::Subscription<control_msgs::msg::MultiDOFCommand>::SharedPtr reference_subscriber_;

  // The system state: this should include the current system velocity and acceleration
  realtime_tools::RealtimeBuffer<std::shared_ptr<control_msgs::msg::MultiDOFCommand>> state_;
  rclcpp::Subscription<control_msgs::msg::MultiDOFCommand>::SharedPtr system_state_subscriber_;
  std::vector<double> system_state_values_;

  // Publish the controller state
  rclcpp::Publisher<control_msgs::msg::MultiDOFStateStamped>::SharedPtr controller_state_publisher_;
  using RTControllerStatePublisher = realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>;
  std::unique_ptr<RTControllerStatePublisher> rt_controller_state_publisher_;

  // Handle updates to the controller parameters
  void update_parameters();
  controller_interface::CallbackReturn configure_parameters();

  // generate_parameter_library members
  // std::shared_ptr<ParamListener> param_listener_;
  // integral_sliding_mode_controller::Params params_;

  // Controller gains
  Eigen::Matrix6d proportional_gain_;
  double sliding_gain_;
  double boundary_thickness_;

  // Error terms
  Eigen::Vector6d initial_velocity_error_;
  Eigen::Vector6d initial_acceleration_error_;
  Eigen::Vector6d total_velocity_error_;

  // Hydrodynamic model
  hydrodynamics::Inertia inertia_;
  hydrodynamics::Coriolis coriolis_;
  hydrodynamics::Damping damping_;
  hydrodynamics::RestoringForces restoring_forces_;

private:
  VELOCITY_CONTROLLERS_LOCAL
  void reference_state_callback(std::shared_ptr<control_msgs::msg::MultiDOFCommand> msg);
};

}  // namespace velocity_controllers
