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

#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "auv_control_msgs/msg/multi_actuator_state_stamped.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hydrodynamics/eigen.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// auto-generated by generate_parameter_library
#include "thruster_allocation_matrix_controller_parameters.hpp"

namespace thruster_allocation_matrix_controller
{

/// Controller used to convert wrench values into thruster forces using a thruster allocation matrix.
class ThrusterAllocationMatrixController : public controller_interface::ChainableControllerInterface
{
public:
  ThrusterAllocationMatrixController() = default;

  auto on_init() -> controller_interface::CallbackReturn override;

  auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto on_set_chained_mode(bool chained_mode) -> bool override;

protected:
  auto on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  void update_parameters();

  auto configure_parameters() -> controller_interface::CallbackReturn;

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Wrench> reference_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Wrench>> reference_sub_;

  std::shared_ptr<rclcpp::Publisher<auv_control_msgs::msg::MultiActuatorStateStamped>> controller_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<auv_control_msgs::msg::MultiActuatorStateStamped>>
    rt_controller_state_pub_;

  std::shared_ptr<thruster_allocation_matrix_controller::ParamListener> param_listener_;
  thruster_allocation_matrix_controller::Params params_;

  std::vector<std::string> thruster_names_;
  Eigen::MatrixXd tam_;
  std::size_t num_thrusters_;

private:
  static constexpr std::size_t DOF = 6;
  std::array<std::string, DOF> dof_names_{"x", "y", "z", "rx", "ry", "rz"};
};

}  // namespace thruster_allocation_matrix_controller
