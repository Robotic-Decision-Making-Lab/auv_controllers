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
  NO_SOLUTION,
  TRANSFORM_ERROR,
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

  /// Solve the inverse kinematics problem for the given target pose, given the current joint configuration.
  [[nodiscard]] auto solve(
    const rclcpp::Duration & period,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const Eigen::VectorXd & q) -> std::expected<trajectory_msgs::msg::JointTrajectoryPoint, SolverError>;

protected:
  /// Update the Pinocchio data
  auto update_pinocchio(const Eigen::VectorXd & q) const -> void;

  /// Transform a target pose into the appropriate frame for the solver.
  [[nodiscard]] auto transform_target_pose(const geometry_msgs::msg::PoseStamped & target_pose) const
    -> geometry_msgs::msg::PoseStamped;

  /// Private method to solve the IK problem, which is called by the public API.
  [[nodiscard]] virtual auto solve_ik(
    const rclcpp::Duration & period,
    const Eigen::Affine3d & target_pose,
    const Eigen::VectorXd & q) -> std::expected<Eigen::VectorXd, SolverError> = 0;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string world_frame_, ee_frame_;

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
};

}  // namespace ik_solvers
