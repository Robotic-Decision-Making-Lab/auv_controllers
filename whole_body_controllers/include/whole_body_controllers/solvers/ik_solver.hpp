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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <expected>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace ik_solvers
{

/// Error codes for the inverse kinematics solvers.
enum class SolverError : std::uint8_t
{
  // No solution was found for the IK problem using the current solver.
  NO_SOLUTION,

  // The target pose could not be transformed into the world frame.
  TRANSFORM_ERROR,

  // The solver encountered an error while solving the IK problem.
  SOLVER_ERROR
};

/// Base class for inverse kinematics solvers.
class IKSolver
{
public:
  /// Constructor.
  IKSolver() = default;

  /// Destructor.
  virtual ~IKSolver() = default;

  /// Initialize the solver.
  virtual auto initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::shared_ptr<pinocchio::Model> & model,
    const std::shared_ptr<pinocchio::Data> & data) -> void;

  /// Solve the inverse kinematics problem for a target pose, given the current integration period and joint
  /// configuration.
  [[nodiscard]] auto solve(
    const rclcpp::Duration & period,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const Eigen::VectorXd & q) -> std::expected<trajectory_msgs::msg::JointTrajectoryPoint, SolverError>;

protected:
  /// Solve the IK problem for the given target pose and joint configuration.
  ///
  /// This is wrapped by the public API. The public API handles the transformation of the target pose into the
  /// appropriate frame and converts of the result into a `JointTrajectoryPoint`. This method only needs to compute
  /// the IK solution.
  [[nodiscard]] virtual auto solve_ik(const Eigen::Affine3d & target_pose, const Eigen::VectorXd & q)
    -> std::expected<Eigen::VectorXd, SolverError> = 0;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string world_frame_, ee_frame_;

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;

private:
  /// Update the Pinocchio data given the current joint configuration.
  auto update_pinocchio(const Eigen::VectorXd & q) const -> void;

  /// Transform a target end effector pose into the appropriate frame for the solver.
  [[nodiscard]] auto transform_target_pose(const geometry_msgs::msg::PoseStamped & target_pose) const
    -> geometry_msgs::msg::PoseStamped;
};

}  // namespace ik_solvers
