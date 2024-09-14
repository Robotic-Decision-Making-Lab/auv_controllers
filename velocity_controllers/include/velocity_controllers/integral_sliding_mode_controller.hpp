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
#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hydrodynamics/eigen.hpp"
#include "hydrodynamics/hydrodynamics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// auto-generated by generate_parameter_library
#include "integral_sliding_mode_controller_parameters.hpp"

namespace velocity_controllers
{

/// Integral sliding mode controller (ISMC) for velocity control of an autonomous underwater vehicle.
class IntegralSlidingModeController : public controller_interface::ChainableControllerInterface
{
public:
  IntegralSlidingModeController() = default;

  auto on_init() -> controller_interface::CallbackReturn override;

  auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto on_cleanup(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto on_set_chained_mode(bool chained_mode) -> bool override;

protected:
  auto on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto update_system_state_values() -> controller_interface::return_type;

  auto update_parameters() -> void;

  controller_interface::CallbackReturn configure_parameters();

  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> reference_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> reference_sub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> system_state_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> system_state_sub_;
  std::vector<double> system_state_values_;

  // We need the system rotation from the inertial frame to the vehicle frame for the hydrodynamic model.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string vehicle_frame_id_;
  std::string inertial_frame_id_;
  realtime_tools::RealtimeBuffer<Eigen::Quaterniond> system_rotation_;

  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::MultiDOFStateStamped>> controller_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>> rt_controller_state_pub_;

  std::shared_ptr<integral_sliding_mode_controller::ParamListener> param_listener_;
  integral_sliding_mode_controller::Params params_;

  Eigen::Matrix6d proportional_gain_;
  double sliding_gain_;
  double boundary_thickness_;

  bool first_update_{true};
  Eigen::Vector6d initial_velocity_error_;
  Eigen::Vector6d total_velocity_error_;

  std::unique_ptr<hydrodynamics::Inertia> inertia_;
  std::unique_ptr<hydrodynamics::Coriolis> coriolis_;
  std::unique_ptr<hydrodynamics::Damping> damping_;
  std::unique_ptr<hydrodynamics::RestoringForces> restoring_forces_;

private:
  // Can't mark an array of strings with constexpr, so we just keep it private
  static constexpr std::size_t DOF = 6;
  std::array<std::string, DOF> dof_names_{"x", "y", "z", "rx", "ry", "rz"};
};

}  // namespace velocity_controllers
