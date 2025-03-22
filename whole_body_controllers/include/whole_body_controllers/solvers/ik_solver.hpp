#pragma once

#include <Eigen/Geometry>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace ik_solvers
{

/// Base class for inverse kinematics solvers.
class IKSolver
{
public:
  /// Constructor.
  IKSolver() = default;

  /// Destructor.
  virtual ~IKSolver() = default;

  /// Solve the inverse kinematics problem for the given target pose, given the current joint configuration.
  [[nodiscard]] virtual auto solve(
    const rclcpp::Duration & period,
    const Eigen::Affine3d & target_pose,
    const Eigen::VectorXd & q) const -> trajectory_msgs::msg::JointTrajectoryPoint = 0;

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
};

}  // namespace ik_solvers
