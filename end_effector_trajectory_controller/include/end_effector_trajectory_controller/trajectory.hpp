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

#include <Eigen/Dense>
#include <chrono>
#include <expected>
#include <optional>

#include "auv_control_msgs/msg/end_effector_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace end_effector_trajectory_controller
{

enum class SampleError : std::uint8_t
{
  SAMPLE_TIME_BEFORE_START,
  SAMPLE_TIME_AFTER_END,
  EMPTY_TRAJECTORY,
};

class Trajectory
{
public:
  Trajectory() = default;

  /// Constructor.
  Trajectory(
    const std::shared_ptr<auv_control_msgs::msg::EndEffectorTrajectory> & trajectory,
    const geometry_msgs::msg::Pose & start_state);

  /// Whether or not the trajectory is empty.
  auto empty() -> bool;

  /// Get the starting time of the trajectory.
  auto start_time() -> rclcpp::Time;

  /// Get the ending time of the trajectory.
  auto end_time() -> rclcpp::Time;

  /// Get the first point in the trajectory.
  auto start_point() -> std::optional<geometry_msgs::msg::Pose>;

  /// Get the last point in the trajectory.
  auto end_point() -> std::optional<geometry_msgs::msg::Pose>;

  /// Sample a point in the trajectory at the given time.
  auto sample(const rclcpp::Time & sample_time) -> std::expected<geometry_msgs::msg::Pose, SampleError>;

private:
  std::shared_ptr<auv_control_msgs::msg::EndEffectorTrajectory> points_;
  rclcpp::Time initial_time_;
  geometry_msgs::msg::Pose initial_state_;
};

}  // namespace end_effector_trajectory_controller
