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

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace topic_sensors
{

class OdomSensor : public hardware_interface::SensorInterface
{
public:
  OdomSensor() = default;

  auto on_init(const hardware_interface::HardwareInfo & info) -> hardware_interface::CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> hardware_interface::CallbackReturn override;

  auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;

  auto read(const rclcpp::Time & time, const rclcpp::Duration & period) -> hardware_interface::return_type override;

private:
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> state_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> state_sub_;

  std::array<std::string, 7> pose_dofs_{"x", "y", "z", "qx", "qy", "qz", "qw"};
  std::array<std::string, 6> twist_dofs_{"vx", "vy", "vz", "wx", "wy", "wz"};
  std::array<double, 13> state_values_;  // we can't use an object with dynamic memory allocation for the state values

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Logger logger_{rclcpp::get_logger("odom_sensor")};
  std::string prefix_;
};

}  // namespace topic_sensors
