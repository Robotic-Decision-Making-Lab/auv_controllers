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

#include "controller_common/common.hpp"

#include <cmath>
#include <limits>
#include <ranges>

namespace common
{

namespace
{

auto nan = std::numeric_limits<double>::quiet_NaN();

}  // namespace

namespace messages
{

auto to_vector(const geometry_msgs::msg::Pose & pose) -> std::vector<double>
{
  return {
    pose.position.x,
    pose.position.y,
    pose.position.z,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w};
}

auto to_vector(const geometry_msgs::msg::Twist & twist) -> std::vector<double>
{
  return {twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z};
}

auto to_vector(const geometry_msgs::msg::Wrench & wrench) -> std::vector<double>
{
  return {wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z};
}

auto to_vector(const nav_msgs::msg::Odometry & odom) -> std::vector<double>
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

auto to_msg(const std::vector<double> & data, geometry_msgs::msg::Pose * msg) -> void
{
  msg->position.x = data[0];
  msg->position.y = data[1];
  msg->position.z = data[2];
  msg->orientation.x = data[3];
  msg->orientation.y = data[4];
  msg->orientation.z = data[5];
  msg->orientation.w = data[6];
}

auto to_msg(const std::vector<double> & data, geometry_msgs::msg::Twist * msg) -> void
{
  msg->linear.x = data[0];
  msg->linear.y = data[1];
  msg->linear.z = data[2];
  msg->angular.x = data[3];
  msg->angular.y = data[4];
  msg->angular.z = data[5];
}

auto reset_message(geometry_msgs::msg::Pose * msg) -> void
{
  msg->position.x = nan;
  msg->position.y = nan;
  msg->position.z = nan;
  msg->orientation.x = nan;
  msg->orientation.y = nan;
  msg->orientation.z = nan;
  msg->orientation.w = nan;
}

auto reset_message(geometry_msgs::msg::Twist * msg) -> void
{
  msg->linear.x = nan;
  msg->linear.y = nan;
  msg->linear.z = nan;
  msg->angular.x = nan;
  msg->angular.y = nan;
  msg->angular.z = nan;
}

auto reset_message(geometry_msgs::msg::Wrench * msg) -> void
{
  msg->force.x = nan;
  msg->force.y = nan;
  msg->force.z = nan;
  msg->torque.x = nan;
  msg->torque.y = nan;
  msg->torque.z = nan;
}

auto reset_message(nav_msgs::msg::Odometry * msg) -> void
{
  msg->pose.pose.position.x = nan;
  msg->pose.pose.position.y = nan;
  msg->pose.pose.position.z = nan;
  msg->pose.pose.orientation.x = nan;
  msg->pose.pose.orientation.y = nan;
  msg->pose.pose.orientation.z = nan;
  msg->pose.pose.orientation.w = nan;
  msg->twist.twist.linear.x = nan;
  msg->twist.twist.linear.y = nan;
  msg->twist.twist.linear.z = nan;
  msg->twist.twist.angular.x = nan;
  msg->twist.twist.angular.y = nan;
  msg->twist.twist.angular.z = nan;
}

}  // namespace messages

namespace math
{

auto calculate_error(const std::vector<double> & reference, const std::vector<double> & state) -> std::vector<double>
{
  std::vector<double> error;
  error.reserve(reference.size());
  std::ranges::transform(reference, state, std::back_inserter(error), [](double ref, double state) {  // NOLINT
    return !std::isnan(ref) && !std::isnan(state) ? ref - state : std::numeric_limits<double>::quiet_NaN();
  });
  return error;
}

}  // namespace math

}  // namespace common
