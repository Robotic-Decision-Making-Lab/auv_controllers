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

#include "end_effector_trajectory_controller/trajectory.hpp"

#include <optional>
#include <ranges>

#include "controller_common/common.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace end_effector_trajectory_controller
{

namespace
{

/// Linear interpolation between two positions.
auto lerp(double start_pos, double end_pos, double t) -> double { return start_pos + t * (end_pos - start_pos); }

/// Spherical linear interpolation between two quaternions.
///
/// See the following for more information:
/// https://www.mathworks.com/help/fusion/ref/quaternion.slerp.html
auto slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2, double t) -> Eigen::Quaterniond
{
  Eigen::Quaterniond result;

  q1.normalize();
  q2.normalize();

  // if the quaternions are very close, just linearly interpolate between them to avoid numerical instability
  const double dot = q1.dot(q2);
  if (common::math::isclose(dot, 1.0, 1e-5, 5e-5)) {
    result.coeffs() = q1.coeffs() + t * (q2.coeffs() - q1.coeffs());
    result.normalize();
    return result;
  }

  const Eigen::Vector4d q1_vec(q1.w(), q1.x(), q1.y(), q1.z());
  const Eigen::Vector4d q2_vec(q2.w(), q2.x(), q2.y(), q2.z());

  const double theta = std::acos(q1.dot(q2));
  const double coeff0 = std::sin((1 - t) * theta) / std::sin(theta);
  const double coeff1 = std::sin(t * theta) / std::sin(theta);

  const Eigen::Vector4d q_vec = (coeff0 * q1_vec) + (coeff1 * q2_vec);

  result.coeffs() = q_vec;
  result.normalize();

  return result;
}

auto interpolate(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2,
  const rclcpp::Time & t0,
  const rclcpp::Time & t1,
  const rclcpp::Time & sample_time) -> geometry_msgs::msg::Pose
{
  const rclcpp::Duration time_from_start = sample_time - t0;
  const rclcpp::Duration time_between_points = t1 - t0;
  const double t = time_from_start.seconds() / time_between_points.seconds();

  // linearly interpolate between the positions
  geometry_msgs::msg::Pose out;
  out.position.x = lerp(p1.position.x, p2.position.x, t);
  out.position.y = lerp(p1.position.y, p2.position.y, t);
  out.position.z = lerp(p1.position.z, p2.position.z, t);

  Eigen::Quaterniond q1, q2;  // NOLINT
  tf2::fromMsg(p1.orientation, q1);
  tf2::fromMsg(p2.orientation, q2);

  // spherical linear interpolation between the orientations
  const Eigen::Quaterniond q_out = slerp(q1, q2, t);
  out.orientation = tf2::toMsg(q_out);

  return out;
}

}  // namespace

Trajectory::Trajectory(
  const auv_control_msgs::msg::EndEffectorTrajectory & trajectory,
  const geometry_msgs::msg::Pose & state)
: points_(std::move(trajectory)),
  initial_time_(static_cast<rclcpp::Time>(trajectory.header.stamp)),
  initial_state_(state)
{
}

auto Trajectory::empty() const -> bool { return points_.points.empty(); }

auto Trajectory::start_time() const -> rclcpp::Time
{
  return empty() ? rclcpp::Time(0) : initial_time_ + points_.points.front().time_from_start;
}

auto Trajectory::end_time() const -> rclcpp::Time
{
  return empty() ? rclcpp::Time(0) : initial_time_ + points_.points.back().time_from_start;
}

auto Trajectory::start_point() const -> std::optional<geometry_msgs::msg::Pose>
{
  if (empty()) {
    return std::nullopt;
  }
  return points_.points.front().point;
}

auto Trajectory::end_point() const -> std::optional<geometry_msgs::msg::Pose>
{
  if (empty()) {
    return std::nullopt;
  }
  return points_.points.back().point;
}

auto Trajectory::sample(const rclcpp::Time & sample_time) const -> std::expected<geometry_msgs::msg::Pose, SampleError>
{
  if (empty()) {
    return std::unexpected(SampleError::EMPTY_TRAJECTORY);
  }

  // the sample time is before the timestamp in the trajectory header
  if (sample_time < initial_time_) {
    return std::unexpected(SampleError::SAMPLE_TIME_BEFORE_START);
  }

  // the sample time is before the first point in the trajectory, so we need to interpolate between the starting
  // state and the first point in the trajectory
  if (sample_time < start_time()) {
    return interpolate(initial_state_, start_point().value(), initial_time_, start_time(), sample_time);
  }

  for (const auto [p1, p2] : std::views::zip(points_.points, points_.points | std::views::drop(1))) {
    const rclcpp::Time t0 = initial_time_ + p1.time_from_start;
    const rclcpp::Time t1 = initial_time_ + p2.time_from_start;

    if (sample_time >= t0 && sample_time <= t1) {
      return interpolate(p1.point, p2.point, t0, t1, sample_time);
    }
  }

  // the whole trajectory has been sampled
  return std::unexpected(SampleError::SAMPLE_TIME_AFTER_END);
}

auto Trajectory::reset_initial_state(const geometry_msgs::msg::Pose & state) -> void { initial_state_ = state; }

}  // namespace end_effector_trajectory_controller
