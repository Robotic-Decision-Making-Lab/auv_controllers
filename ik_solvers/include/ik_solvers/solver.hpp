#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <expected>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace ik_solvers
{

/// Error codes for the inverse kinematics solvers.
enum class SolverError : std::uint8_t
{
  NO_SOLUTION,
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

  /// Solve the IK problem for a target pose given the integration period and current joint configuration.
  [[nodiscard]] auto solve(const rclcpp::Duration & period, const Eigen::Affine3d & goal, const Eigen::VectorXd & q)
    -> std::expected<trajectory_msgs::msg::JointTrajectoryPoint, SolverError>;

protected:
  /// Solve the IK problem for the given target pose and joint configuration.
  ///
  /// This is wrapped by the public API. The public API handles updating pinocchio for Jacobian calculation and converts
  /// of the result into a `JointTrajectoryPoint`. This method only needs to compute the IK solution.
  [[nodiscard]] virtual auto solve_ik(const Eigen::Affine3d & goal, const Eigen::VectorXd & q)
    -> std::expected<Eigen::VectorXd, SolverError> = 0;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  std::string ee_frame_;

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;

private:
  auto update_pinocchio(const Eigen::VectorXd & q) const -> void;
};

}  // namespace ik_solvers
