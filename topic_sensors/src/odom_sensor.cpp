// Copyright 2025, Akshaya Agrawal, Evan Palmer
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

#include "topic_sensors/odom_sensor.hpp"

#include <format>
#include <ranges>
#include <tuple>

#include "controller_common/common.hpp"

namespace topic_sensors
{

auto OdomSensor::on_init(const hardware_interface::HardwareComponentInterfaceParams & /*info*/)
  -> hardware_interface::CallbackReturn
{
  prefix_ = info_.hardware_parameters.at("prefix");
  rclcpp::NodeOptions options;
  options.arguments({std::format("--ros-args -r __ns:={} -r __node:=odom_sensor_{}", prefix_, info_.name)});
  node_ = rclcpp::Node::make_shared("_", options);
  executor_.add_node(node_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto OdomSensor::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) -> hardware_interface::CallbackReturn
{
  nav_msgs::msg::Odometry initial_state;
  common::messages::reset_message(&initial_state);
  state_.set(initial_state);

  // the state interfaces are exported by the framework after on_init, so the handles are available here
  std::size_t idx = 0;
  for (const auto & dof : pose_dofs_) {
    const auto prefix = prefix_.empty() ? dof : std::format("{}/{}", prefix_, dof);
    state_handles_[idx++] =
      get_state_interface_handle(std::format("{}/{}", prefix, hardware_interface::HW_IF_POSITION));
  }
  for (const auto & dof : twist_dofs_) {
    const auto prefix = prefix_.empty() ? dof : std::format("{}/{}", prefix_, dof);
    state_handles_[idx++] =
      get_state_interface_handle(std::format("{}/{}", prefix, hardware_interface::HW_IF_VELOCITY));
  }

  const std::string topic = info_.hardware_parameters.at("topic");
  if (topic.empty()) {
    RCLCPP_ERROR(logger_, "Topic name is empty. Please provide a valid topic name.");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Subscribing to topic: %s", topic.c_str());  // NOLINT

  state_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic,
    rclcpp::SensorDataQoS(),
    [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {  // NOLINT
      state_.set(*msg);
    });
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto OdomSensor::export_unlisted_state_interface_descriptions() -> std::vector<hardware_interface::InterfaceDescription>
{
  std::vector<hardware_interface::InterfaceDescription> descriptions;
  descriptions.reserve(pose_dofs_.size() + twist_dofs_.size());

  auto add_description = [this, &descriptions](const std::string & dof, const std::string & interface) -> void {
    hardware_interface::InterfaceInfo info;
    info.name = interface;
    descriptions.emplace_back(prefix_.empty() ? dof : std::format("{}/{}", prefix_, dof), info);
  };

  for (const auto & dof : pose_dofs_) {
    add_description(dof, hardware_interface::HW_IF_POSITION);
  }
  for (const auto & dof : twist_dofs_) {
    add_description(dof, hardware_interface::HW_IF_VELOCITY);
  }

  return descriptions;
}

auto OdomSensor::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hardware_interface::return_type
{
  if (rclcpp::ok()) {
    executor_.spin_some();
    state_.try_get([this](const nav_msgs::msg::Odometry & state) -> void {
      for (const auto & [handle, value] : std::views::zip(state_handles_, common::messages::to_vector(state))) {
        std::ignore = set_state(handle, value, false);
      }
    });
  }
  return hardware_interface::return_type::OK;
}

}  // namespace topic_sensors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(topic_sensors::OdomSensor, hardware_interface::SensorInterface)
