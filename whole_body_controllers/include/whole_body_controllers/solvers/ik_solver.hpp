#pragma once

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace ik_solvers
{

class IKSolver
{
public:
  IKSolver() = default;

  virtual ~IKSolver() = default;

  [[nodiscard]] virtual auto solve(const rclcpp::Duration & period) const
    -> trajectory_msgs::msg::JointTrajectoryPoint = 0;

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
};

}  // namespace ik_solvers
