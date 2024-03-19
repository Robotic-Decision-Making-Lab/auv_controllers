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

#include "mock_hardware/mock_hardware.hpp"

#include "hardware_interface/lexical_casts.hpp"

namespace mock_hardware
{

hardware_interface::CallbackReturn MockHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MockHardware"), "Initializing MockHardware interface for ros2_control.");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(  // NOLINT
      rclcpp::get_logger("MockHardware"), "Failed to initialize the MockHardware system interface.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  auto populate_non_standard_interfaces = [this](auto interface_list, auto & non_standard_interfaces) {
    for (const auto & interface : interface_list) {
      // add to list if non-standard interface
      if (
        std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end()) {
        if (
          std::find(non_standard_interfaces.begin(), non_standard_interfaces.end(), interface.name) ==
          non_standard_interfaces.end()) {
          non_standard_interfaces.emplace_back(interface.name);
        }
      }
    }
  };

  // Initialize storage for standard interfaces
  initialize_storage_vectors(joint_commands_, joint_states_, standard_interfaces_, info_.joints);

  // set all values without initial values to 0
  for (auto i = 0U; i < info_.joints.size(); i++) {
    for (auto j = 0U; j < standard_interfaces_.size(); j++) {
      if (std::isnan(joint_states_[j][i])) {
        joint_states_[j][i] = 0.0;
      }
    }
  }

  // search for non-standard joint interfaces
  for (const auto & joint : info_.joints) {
    // populate non-standard command interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.command_interfaces, other_interfaces_);

    // populate non-standard state interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.state_interfaces, other_interfaces_);
  }

  // Initialize storage for non-standard interfaces
  initialize_storage_vectors(other_commands_, other_states_, other_interfaces_, info_.joints);

  // Set effort control mode per default
  joint_control_mode_.resize(info_.joints.size(), EFFORT_INTERFACE_INDEX);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MockHardware"), "Successfully initialized the MockHardware system interface!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockHardware::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MockHardware"), "Successfully configured the MockHardware system interface");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MockHardware"), "Shutting down the MockHarwadre system interface.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockHardware::on_activate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MockHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joints' state interfaces
  for (auto i = 0U; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.state_interfaces) {
      if (!get_interface(joint.name, standard_interfaces_, interface.name, i, joint_states_, state_interfaces)) {
        if (!get_interface(joint.name, other_interfaces_, interface.name, i, other_states_, state_interfaces)) {
          throw std::runtime_error(
            "Interface is not found in the standard nor other list. "
            "This should never happen!");
        }
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MockHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joints' command interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.command_interfaces) {
      if (!get_interface(joint.name, standard_interfaces_, interface.name, i, joint_commands_, command_interfaces)) {
        if (!get_interface(joint.name, other_interfaces_, interface.name, i, other_commands_, command_interfaces)) {
          throw std::runtime_error(
            "Interface is not found in the standard nor other list. "
            "This should never happen!");
        }
      }
    }
  }
  return command_interfaces;
}

hardware_interface::return_type MockHardware::prepare_command_mode_switch(
  const std::vector<std::string> & /* start interfaces */, const std::vector<std::string> & /* stop interfaces */)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockHardware::perform_command_mode_switch(
  const std::vector<std::string> & /* start interfaces */, const std::vector<std::string> & /* stop interfaces */)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

template <typename HandleType>
bool MockHardware::get_interface(
  const std::string & name, const std::vector<std::string> & interface_list, const std::string & interface_name,
  const size_t vector_index, std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces)
{
  auto it = std::find(interface_list.begin(), interface_list.end(), interface_name);
  if (it != interface_list.end()) {
    auto j = std::distance(interface_list.begin(), it);
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

void MockHardware::initialize_storage_vectors(  // NOLINT
  std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
  const std::vector<std::string> & interfaces, const std::vector<hardware_interface::ComponentInfo> & component_infos)
{
  // Initialize storage for all joints, regardless of their existence
  commands.resize(interfaces.size());
  states.resize(interfaces.size());
  for (auto i = 0U; i < interfaces.size(); i++) {
    commands[i].resize(component_infos.size(), std::numeric_limits<double>::quiet_NaN());
    states[i].resize(component_infos.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Initialize with values from URDF
  for (auto i = 0U; i < component_infos.size(); i++) {
    const auto & component = component_infos[i];
    for (const auto & interface : component.state_interfaces) {
      auto it = std::find(interfaces.begin(), interfaces.end(), interface.name);

      // If interface name is found in the interfaces list
      if (it != interfaces.end()) {
        auto index = std::distance(interfaces.begin(), it);

        // Check the initial_value param is used
        if (!interface.initial_value.empty()) {
          states[index][i] = hardware_interface::stod(interface.initial_value);
        }
      }
    }
  }
}

}  // namespace mock_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mock_hardware::MockHardware, hardware_interface::SystemInterface)
