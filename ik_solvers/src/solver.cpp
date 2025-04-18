#include "ik_solvers/solver.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

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
}

auto IKSolver::update_pinocchio(const Eigen::VectorXd & q) const -> void
{
  pinocchio::forwardKinematics(*model_, *data_, q);
  pinocchio::updateFramePlacements(*model_, *data_);
  pinocchio::computeJointJacobians(*model_, *data_);
}

auto IKSolver::solve(const rclcpp::Duration & period, const Eigen::Affine3d & goal, const Eigen::VectorXd & q)
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
  point.velocities.reserve(solution.size());

  point.positions = std::vector<double>(q_next.data(), q_next.data() + q_next.size());
  point.velocities = std::vector<double>(solution.data(), solution.data() + solution.size());

  return point;
}

}  // namespace ik_solvers
