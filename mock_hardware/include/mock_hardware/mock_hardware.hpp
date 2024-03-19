// Copyright 2024, Everardo Gonzalez, Rakesh Vivekanandan
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

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "mock_hardware/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace mock_hardware
{

/**
 * @brief Mock hardware interface that takes a wrench vector as input and outputs commands to CLI
 */

static constexpr size_t POSITION_INTERFACE_INDEX = 0;
static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;
static constexpr size_t ACCELERATION_INTERFACE_INDEX = 2;
static constexpr size_t EFFORT_INTERFACE_INDEX = 3;
static constexpr size_t PWM_INTERFACE_INDEX = 4;

class MockHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MockHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  template <typename HandleType>
  bool get_interface(
    const std::string & name, const std::vector<std::string> & interface_list, const std::string & interface_name,
    size_t vector_index, std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

  void initialize_storage_vectors(
    std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
    const std::vector<std::string> & interfaces,
    const std::vector<hardware_interface::ComponentInfo> & component_infos);

  const std::vector<std::string> standard_interfaces_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION,
    hardware_interface::HW_IF_EFFORT};

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<std::vector<double>> joint_states_;

  std::vector<std::string> other_interfaces_;
  /// The size of this vector is (other_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> other_commands_;
  std::vector<std::vector<double>> other_states_;

  std::vector<size_t> joint_control_mode_;
};

}  // namespace mock_hardware
