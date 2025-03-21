#pragma once

#include <concepts>
#include <cstdint>
#include <functional>

#include "ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ik_solvers
{

namespace hierarchy
{

/// Base class for constraints/tasks. This is equivalent to an equality constraint.
class Constraint
{
public:
  /// Create a new constraint given the task priority and feedback gain.
  Constraint(int priority, double gain)
  : priority_(priority),
    gain_(gain){};

  /// Destructor.
  virtual ~Constraint() = default;

  /// Compute the Jacobian for the constraint.
  [[nodiscard]] virtual auto jacobian() const -> Eigen::MatrixXd = 0;

  [[nodiscard]] virtual auto error() const -> Eigen::VectorXd = 0;

  /// Get the priority of the constraint.
  [[nodiscard]] auto priority() const -> int { return priority_; }

  /// Get the feedback gain for the constraint.
  [[nodiscard]] auto gain() const -> double { return gain_; }

protected:
  int priority_;
  double gain_;
};

/// Base class for set constraints/tasks. This is equivalent to an inequality constraint.
class SetConstraint : public Constraint
{
public:
  /// Create a new set constraint given the constraint bounds, tolerance, activation threshold, task priority,and gain.
  SetConstraint(double ub, double lb, double tol, double activation_thresh, int priority, double gain)
  : Constraint(priority, gain)
  {
    upper_limit_ = ub;
    lower_limit_ = lb;
    tolerance_ = tol;
    activation_threshold_ = activation_thresh;
  }

  /// Check whether or not the constraint is active. Set constraints are only active when the current value is within
  /// the activation threshold for the task.
  [[nodiscard]] virtual auto is_active() const -> bool = 0;

  /// Get the upper limit of the constraint.
  [[nodiscard]] auto upper_limit() const -> double { return upper_limit_; }

  /// Get the lower limit of the constraint.
  [[nodiscard]] auto lower_limit() const -> double { return lower_limit_; }

  /// Get the tolerance of the constraint.
  [[nodiscard]] auto tolerance() const -> double { return tolerance_; }

  /// Get the activation threshold of the constraint.
  [[nodiscard]] auto activation_threshold() const -> double { return activation_threshold_; }

protected:
  bool is_active_{false};
  double upper_limit_;
  double lower_limit_;
  double tolerance_;
  double activation_threshold_;
};

/// Comparator used to sort constraints based on their priority.
/// If the priorities are equal, the constraints are sorted based on their memory address.
struct ConstraintCompare
{
  bool operator()(const std::shared_ptr<Constraint> & lhs, const std::shared_ptr<Constraint> & rhs) const
  {
    return lhs->priority() == rhs->priority() ? lhs.get() < rhs.get() : lhs->priority() < rhs->priority();
  }
};

/// Type alias for a set of constraints.
using ConstraintSet = std::set<std::shared_ptr<Constraint>, ConstraintCompare>;

/// Class used to manage the task hierarchy.
class TaskHierarchy
{
public:
  /// Constructor.
  TaskHierarchy() = default;

  /// Insert a new constraint into the hierarchy. This will automatically sort the constraints based on their priority.
  /// For now, only equality constraints and high priority set constraints are supported.
  auto insert(const std::shared_ptr<Constraint> & constraint) -> void;

  /// Clear all constraints from the hierarchy.
  auto clear() -> void;

  /// Get the set of all active inequality constraints in the hierarchy.
  [[nodiscard]] auto set_constraints() const -> ConstraintSet;

  /// Get the set of all equality constraints in the hierarchy.
  [[nodiscard]] auto equality_constraints() const -> ConstraintSet;

  /// Get the set of all potential hierarchies for the active tasks.
  [[nodiscard]] auto hierarchies() const -> std::vector<ConstraintSet>;

private:
  ConstraintSet constraints_;
};

// TODO(evan-palmer): Figure this out
class PoseConstraint : public Constraint
{
public:
  PoseConstraint(int priority, double gain);
};

class JointLimitConstraint : public SetConstraint
{
public:
  JointLimitConstraint(Eigen::VectorXd ub, Eigen::VectorXd lb, Eigen::VectorXd tolerance, int priority, double gain);
};

}  // namespace hierarchy

class TaskPriorityIKSolver : public IKSolver
{
public:
  TaskPriorityIKSolver() = default;

  [[nodiscard]] auto solve(rclcpp::Duration period) const -> trajectory_msgs::msg::JointTrajectoryPoint override;

  auto add_constraint(const std::shared_ptr<hierarchy::Constraint> & constraint) -> void;

  auto clear_constraints() -> void;

private:
  hierarchy::TaskHierarchy task_hierarchy_;
};

}  // namespace ik_solvers
