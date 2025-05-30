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

#pragma once

#include "auv_control_msgs/msg/ik_controller_state_stamped.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ik_solvers/solver.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// auto-generated by generate_parameter_library
#include <whole_body_controllers/ik_controller_parameters.hpp>

namespace whole_body_controllers
{

class IKController : public controller_interface::ChainableControllerInterface
{
public:
  IKController() = default;

  auto on_init() -> controller_interface::CallbackReturn override;

  // NOLINTNEXTLINE(modernize-use-nodiscard)
  auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  // NOLINTNEXTLINE(modernize-use-nodiscard)
  auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

protected:
  auto on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto update_system_state_values() -> controller_interface::return_type;

  auto update_chained_reference_values() -> controller_interface::return_type;

  auto update_parameters() -> void;

  auto configure_parameters() -> controller_interface::CallbackReturn;

  auto update_and_validate_interfaces() -> controller_interface::return_type;

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Pose> reference_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> reference_sub_;

  // allow users to send the vehicle state as a message - this can simplify integration with state estimators
  // that publish the vehicle state without the manipulator states
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> vehicle_state_sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> vehicle_state_;

  std::vector<double> position_state_values_, velocity_state_values_;

  std::unique_ptr<pluginlib::ClassLoader<ik_solvers::IKSolver>> loader_;
  std::shared_ptr<ik_solvers::IKSolver> solver_;

  std::unique_ptr<ik_controller::ParamListener> param_listener_;
  ik_controller::Params params_;

  std::shared_ptr<rclcpp::Publisher<auv_control_msgs::msg::IKControllerStateStamped>> controller_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<auv_control_msgs::msg::IKControllerStateStamped>>
    rt_controller_state_pub_;

  // store the names of the base joints
  std::vector<std::string> free_flyer_pos_dofs_, free_flyer_vel_dofs_;

  // store the names of the position and velocity interfaces
  // this is stored in the same order as the pinocchio model to simplify integration with the solver
  std::vector<std::string> position_interface_names_, velocity_interface_names_;

  // track the interfaces used by the controller
  bool use_position_commands_, use_velocity_commands_, use_position_states_, use_velocity_states_;
  std::size_t n_command_interfaces_, n_state_interfaces_;

  rclcpp::Logger logger_{rclcpp::get_logger("ik_controller")};
};

}  // namespace whole_body_controllers
