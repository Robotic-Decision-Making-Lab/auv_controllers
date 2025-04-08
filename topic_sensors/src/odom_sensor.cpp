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

namespace topic_sensors
{

namespace
{

// NOLINTNEXTLINE(readability-non-const-parameter)
auto reset_odom_msg(nav_msgs::msg::Odometry * msg) -> void
{
  msg->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
  msg->pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  msg->twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
}

auto odom_to_vector(const nav_msgs::msg::Odometry & odom) -> std::vector<double>
{
  return {
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z,
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w,
    odom.twist.twist.linear.x,
    odom.twist.twist.linear.y,
    odom.twist.twist.linear.z,
    odom.twist.twist.angular.x,
    odom.twist.twist.angular.y,
    odom.twist.twist.angular.z};
}

}  // namespace

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
  reset_odom_msg(state_.readFromNonRT());
  state_values_.resize(pose_dofs_.size() + twist_dofs_.size(), std::numeric_limits<double>::quiet_NaN());

  const std::string topic = info_.hardware_parameters.at("topic");
  if (topic.empty()) {
    RCLCPP_ERROR(logger_, "Topic name is empty. Please provide a valid topic name.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(node_->get_logger(), std::format("Subscribing to topic: {}", topic).c_str());

  state_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SensorDataQoS(), [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {  // NOLINT
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
  }
  const auto * current_state = state_.readFromRT();
  RCLCPP_INFO(logger_, "this is a test");
  // std::ranges::copy(odom_to_vector(*current_state), state_values_.begin());
  return hardware_interface::return_type::OK;
}

}  // namespace topic_sensors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(topic_sensors::OdomSensor, hardware_interface::SensorInterface)
