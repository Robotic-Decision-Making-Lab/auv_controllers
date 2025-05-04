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

  /// Create a new trajectory given a trajectory message and the initial state.
  Trajectory(const auv_control_msgs::msg::EndEffectorTrajectory & trajectory, const geometry_msgs::msg::Pose & state);

  /// Whether or not the trajectory is empty.
  [[nodiscard]] auto empty() const -> bool;

  /// Get the starting time of the trajectory.
  [[nodiscard]] auto start_time() const -> rclcpp::Time;

  /// Get the ending time of the trajectory.
  [[nodiscard]] auto end_time() const -> rclcpp::Time;

  /// Get the first point in the trajectory.
  [[nodiscard]] auto start_point() const -> std::optional<geometry_msgs::msg::Pose>;

  /// Get the last point in the trajectory.
  [[nodiscard]] auto end_point() const -> std::optional<geometry_msgs::msg::Pose>;

  /// Sample a point in the trajectory at the given time.
  [[nodiscard]] auto sample(const rclcpp::Time & sample_time) const
    -> std::expected<geometry_msgs::msg::Pose, SampleError>;

  /// Reset the initial end effector state and trajectory start time.
  auto reset_initial_state(const geometry_msgs::msg::Pose & state) -> void;

private:
  auv_control_msgs::msg::EndEffectorTrajectory points_;
  rclcpp::Time initial_time_;
  geometry_msgs::msg::Pose initial_state_;
};

}  // namespace end_effector_trajectory_controller
