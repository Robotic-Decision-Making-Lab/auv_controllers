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

#include <Eigen/Dense>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"

namespace whole_body_controllers
{

namespace tasks
{

class Task
{
public:
  auto priority() -> int;

  auto gain() -> double;

private:
  int priority_;
  double gain_;
};

class PoseEqualityTask : public Task
{
public:
  PoseEqualityTask() = default;

private:
};

class JointInequalityTask : public Task
{
public:
private:
};

class ManipulabilityTask : public Task
{
public:
private:
};

}  // namespace tasks

class TaskPriorityIKControl : public controller_interface::ChainableControllerInterface
{
public:
  TaskPriorityIKControl() = default;

  auto on_init() -> controller_interface::CallbackReturn override;

  auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

private:
  auto on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto update_system_state_values() -> controller_interface::return_type;

  auto update_parameters() -> void;

  auto configure_parameters() -> controller_interface::CallbackReturn;

  // TODO(evan-palmer): Define static tasks and priorities:
  // We could define the task as a struct to hold its parameters
  // task type, priority, gain, etc.
  // - EE position
  // - EE orientation
  // - Joint limits
  // - Joint velocity limits
  // - Vehicle orientation
  // - Vehicle position

  // TODO(evan-palmer) We need an interface to read the current state of the system
  // TODO(evan-palmer) We need to define the hierarchy of tasks and their combinations for set-based control
};

}  // namespace whole_body_controllers
