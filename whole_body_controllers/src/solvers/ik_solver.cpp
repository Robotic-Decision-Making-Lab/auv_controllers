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

#include "whole_body_controllers/solvers/ik_solver.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ik_solvers
{

auto IKSolver::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::shared_ptr<pinocchio::Model> & model,
  const std::shared_ptr<pinocchio::Data> & data) -> void
{
  node_ = node;
  model_ = model;
  data_ = data;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

auto IKSolver::update_pinocchio(const Eigen::VectorXd & q) const -> void
{
  pinocchio::forwardKinematics(*model_, *data_, q);
  pinocchio::updateFramePlacements(*model_, *data_);
  pinocchio::computeJointJacobians(*model_, *data_);
}

auto IKSolver::transform_target_pose(const geometry_msgs::msg::PoseStamped & target) const
  -> geometry_msgs::msg::PoseStamped
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  const auto transform = tf_buffer_->lookupTransform(world_frame_, target.header.frame_id, target.header.stamp);
  tf2::doTransform(target.pose, transformed_pose.pose, transform);
  transformed_pose.header.frame_id = world_frame_;
  transformed_pose.header.stamp = target.header.stamp;
  return transformed_pose;
}

auto IKSolver::solve(
  const rclcpp::Duration & period,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const Eigen::VectorXd & q) -> std::expected<trajectory_msgs::msg::JointTrajectoryPoint, SolverError>
{
  // update the pinocchio data and model to use the current joint configuration
  update_pinocchio(q);

  // transform the target pose into the world frame
  geometry_msgs::msg::PoseStamped transformed_pose;
  if (target_pose.header.frame_id == world_frame_) {
    transformed_pose = target_pose;
  } else {
    try {
      transformed_pose = transform_target_pose(target_pose);
    }
    catch (const tf2::TransformException & ex) {
      return std::unexpected(SolverError::TRANSFORM_ERROR);
    }
  }

  // convert the message into an Eigen Affine3d for easier manipulation
  Eigen::Affine3d target;
  tf2::fromMsg(transformed_pose.pose, target);

  const auto result = solve_ik(target, q);

  if (!result.has_value()) {
    return std::unexpected(result.error());
  }

  const Eigen::VectorXd solution = result.value();

  // Integrate the solution to get the new joint positions
  const Eigen::VectorXd q_next = pinocchio::integrate(*model_, q, period.seconds() * solution);

  // Convert the result into a JointTrajectoryPoint
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = period;

  point.positions.reserve(q_next.size());
  point.velocities.reserve(solution.size());

  point.positions = std::vector<double>(q_next.data(), q_next.data() + q_next.size());
  point.velocities = std::vector<double>(solution.data(), solution.data() + solution.size());

  return point;
}

}  // namespace ik_solvers
