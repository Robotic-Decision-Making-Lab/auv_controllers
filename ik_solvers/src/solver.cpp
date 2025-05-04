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

#include "ik_solvers/solver.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace ik_solvers
{

auto IKSolver::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::shared_ptr<pinocchio::Model> & model,
  const std::shared_ptr<pinocchio::Data> & data,
  const std::string & /*prefix*/) -> void
{
  node_ = node;
  model_ = model;
  data_ = data;
}

auto IKSolver::update_pinocchio(const Eigen::VectorXd & q) const -> void
{
  pinocchio::forwardKinematics(*model_, *data_, q);
  pinocchio::updateFramePlacements(*model_, *data_);
  pinocchio::computeJointJacobians(*model_, *data_);
}

auto IKSolver::solve(const rclcpp::Duration & period, const Eigen::Isometry3d & goal, const Eigen::VectorXd & q)
  -> std::expected<trajectory_msgs::msg::JointTrajectoryPoint, SolverError>
{
  // update the pinocchio data and model to use the current joint configuration
  update_pinocchio(q);

  const auto result = solve_ik(goal, q);
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
  std::ranges::copy(q_next, std::back_inserter(point.positions));

  point.velocities.reserve(solution.size());
  std::ranges::copy(solution, std::back_inserter(point.velocities));

  return point;
}

}  // namespace ik_solvers
