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

#include <vector>

#include "auv_control_msgs/msg/impedance_command.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace common
{

namespace messages
{

auto to_vector(const geometry_msgs::msg::Pose & pose) -> std::vector<double>;

auto to_vector(const geometry_msgs::msg::Twist & twist) -> std::vector<double>;

auto to_vector(const geometry_msgs::msg::Wrench & wrench) -> std::vector<double>;

auto to_vector(const nav_msgs::msg::Odometry & odom) -> std::vector<double>;

auto to_vector(const auv_control_msgs::msg::ImpedanceCommand & command) -> std::vector<double>;

auto to_msg(const std::vector<double> & data, geometry_msgs::msg::Pose * msg) -> void;

auto to_msg(const std::vector<double> & data, geometry_msgs::msg::Twist * msg) -> void;

auto to_msg(const std::vector<double> & data, geometry_msgs::msg::Wrench * msg) -> void;

auto reset_message(geometry_msgs::msg::Pose * msg) -> void;

auto reset_message(geometry_msgs::msg::Twist * msg) -> void;

auto reset_message(geometry_msgs::msg::Wrench * msg) -> void;

auto reset_message(nav_msgs::msg::Odometry * msg) -> void;

auto reset_message(auv_control_msgs::msg::ImpedanceCommand * msg) -> void;

}  // namespace messages

namespace math
{

auto calculate_error(const std::vector<double> & reference, const std::vector<double> & state) -> std::vector<double>;

auto has_nan(const std::vector<double> & vec) -> bool;

auto all_nan(const std::vector<double> & vec) -> bool;

auto isclose(double a, double b, double rtol = 1e-05, double atol = 1e-08) -> bool;

}  // namespace math

}  // namespace common
