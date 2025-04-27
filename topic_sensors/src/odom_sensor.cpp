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

#include "controller_common/common.hpp"
#include "message_transforms/transforms.hpp"

namespace topic_sensors
{

auto OdomSensor::on_init(const hardware_interface::HardwareInfo & /*info*/) -> hardware_interface::CallbackReturn
{
  prefix_ = info_.hardware_parameters.at("prefix");
  rclcpp::NodeOptions options;
  options.arguments({std::format("--ros-args -r __ns:={} -r __node:=odom_sensor_{}", prefix_, info_.name)});
  node_ = rclcpp::Node::make_shared("_", options);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto OdomSensor::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) -> hardware_interface::CallbackReturn
{
  common::messages::reset_message(state_.readFromNonRT());
  state_values_.fill(std::numeric_limits<double>::quiet_NaN());

  const std::string topic = info_.hardware_parameters.at("topic");
  if (topic.empty()) {
    RCLCPP_ERROR(logger_, "Topic name is empty. Please provide a valid topic name.");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Subscribing to topic: %s", topic.c_str());  // NOLINT

  const bool transform_message = info_.hardware_parameters.at("transform_message") == "true";
  if (transform_message) {
    // NOLINTNEXTLINE
    RCLCPP_INFO(logger_, "Incoming messages will be transform from the ROS mobile standard to the maritime standard");
  }

  state_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SensorDataQoS(), [this, &transform_message](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
      if (transform_message) {
        m2m::transform_message(*msg, "map_ned", "base_link_fsd");
      }
      state_.writeFromNonRT(*msg);
    });
  return hardware_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto OdomSensor::export_state_interfaces() -> std::vector<hardware_interface::StateInterface>
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(pose_dofs_.size() + twist_dofs_.size());

  for (const auto [i, dof] : std::views::enumerate(pose_dofs_)) {
    interfaces.emplace_back(
      prefix_.empty() ? dof : std::format("{}/{}", prefix_, dof),
      hardware_interface::HW_IF_POSITION,
      &state_values_[i]);
  }

  for (const auto [j, dof] : std::views::enumerate(twist_dofs_)) {
    interfaces.emplace_back(
      prefix_.empty() ? dof : std::format("{}/{}", prefix_, dof),
      hardware_interface::HW_IF_VELOCITY,
      &state_values_[pose_dofs_.size() + j]);
  }

  return interfaces;
}

auto OdomSensor::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hardware_interface::return_type
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    const auto * current_state = state_.readFromRT();
    std::ranges::copy(common::messages::to_vector(*current_state), state_values_.begin());
  }
  return hardware_interface::return_type::OK;
}

}  // namespace topic_sensors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(topic_sensors::OdomSensor, hardware_interface::SensorInterface)
