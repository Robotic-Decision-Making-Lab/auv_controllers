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

/// Base class for constraints.
class Constraint
{
public:
  /// Create a new constraint given the current primal value, constraint value, task priority and feedback gain.
  Constraint(Eigen::MatrixXd primal, Eigen::MatrixXd constraint, int priority, double gain)
  : primal_(primal),
    constraint_(constraint),
    priority_(priority),
    gain_(gain){};

  /// Create a new constraint given the constraint value, task priority and feedback gain.
  Constraint(Eigen::MatrixXd constraint, int priority, double gain)
  : primal_(Eigen::MatrixXd::Zero(constraint.size())),
    constraint_(constraint),
    priority_(priority),
    gain_(gain){};

  /// Destructor.
  virtual ~Constraint() = default;

  /// Update the primal value for the constraint.
  [[nodiscard]] auto update_primal(const Eigen::MatrixXd & primal) -> void { primal_ = primal; };

  /// Get the primal value for the constraint.
  [[nodiscard]] auto primal() const -> Eigen::MatrixXd { return primal_; };

  /// Compute the Jacobian for the constraint.
  [[nodiscard]] virtual auto jacobian() const -> Eigen::MatrixXd = 0;

  /// Compute the error between the primal and constraint.
  [[nodiscard]] virtual auto error() const -> Eigen::VectorXd = 0;

  /// Get the constraint priority.
  [[nodiscard]] auto priority() const -> int { return priority_; }

  /// Get the feedback gain.
  [[nodiscard]] auto gain() const -> double { return gain_; }

protected:
  Eigen::MatrixXd primal_;
  Eigen::MatrixXd constraint_;
  int priority_;
  double gain_;
};

/// Base class for scalar set constraints.
class SetConstraint : public Constraint
{
public:
  /// Create a new set constraint given the current primal value, constraint value, constraint bounds, tolerance,
  /// activation threshold, task priority,and gain.
  SetConstraint(double primal, double ub, double lb, double tol, double activation, int priority, double gain)
  : Constraint(Eigen::MatrixXd::Constant(primal), Eigen::MatrixXd::Zero(1), priority, gain),
    upper_limit_(ub),
    lower_limit_(lb),
    upper_safety_(ub - tol),
    lower_safety_(lb + tol),
    upper_threshold_(ub - tol - activation),
    lower_threshold_(lb + tol + activation){};

  /// Create a new set constraint given the constraint value, constraint bounds, tolerance,
  /// activation threshold, task priority,and gain.
  SetConstraint(double ub, double lb, double tol, double activation, int priority, double gain)
  : SetConstraint(0.0, ub, lb, tol, activation, priority, gain){};

  /// Check whether or not the constraint is active. Set constraints are only active when the current value is within
  /// the activation threshold for the task.
  [[nodiscard]] auto is_active() const -> bool
  {
    double primal = primal_.value();
    if (primal < lower_threshold_ || primal > upper_threshold_) {
      return true;
    }
    return false;
  };

  // TODO: Check why this isn't overriding the base class
  [[nodiscard]] auto update_primal(const Eigen::MatrixXd & primal) -> void
  {
    primal_ = primal;
    update_constraint();
  };

  auto update_primal(double primal) -> void { update_primal(Eigen::MatrixXd::Constant(primal)); };

  auto primal() const -> double { return primal_.value(); }

  /// Get the upper limit of the constraint.
  [[nodiscard]] auto upper_limit() const -> double { return upper_limit_; }

  /// Get the lower limit of the constraint.
  [[nodiscard]] auto lower_limit() const -> double { return lower_limit_; }

  /// Get the upper safety limit for the constraint.
  /// The safety value serves as a target point for the algorithm to target when the activation threshold is exceeded.
  [[nodiscard]] auto upper_safety() const -> double { return upper_safety_; }

  /// Get the lower safety limit for the constraint.
  /// The safety value serves as a target point for the algorithm to target when the activation threshold is exceeded.
  [[nodiscard]] auto lower_safety() const -> double { return lower_safety_; }

  /// Get the upper activation threshold for the constraint.
  [[nodiscard]] auto upper_threshold() const -> double { return upper_threshold_; }

  /// Get the lower activation threshold for the constraint.
  [[nodiscard]] auto lower_threshold() const -> double { return lower_threshold_; }

protected:
  auto update_constraint() -> void
  {
    // Set the constraint value based on whether or not the task is active
    double primal = primal_.value();
    if (primal < lower_threshold_) {
      constraint_ = Eigen::MatrixXd::Constant(lower_safety_);
    } else if (primal > upper_threshold_) {
      constraint_ = Eigen::MatrixXd::Constant(upper_safety_);
    } else {
      constraint_ = primal_;
    }
  }

  double upper_limit_, lower_limit_;
  double upper_safety_, lower_safety_;
  double upper_threshold_, lower_threshold_;
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

}  // namespace hierarchy

class TaskPriorityIKSolver : public IKSolver
{
public:
  TaskPriorityIKSolver() = default;

  auto add_constraint(const std::shared_ptr<hierarchy::Constraint> & constraint) -> void
  {
    task_hierarchy_.insert(constraint);
  };

  auto clear_constraints() -> void { task_hierarchy_.clear(); };

  [[nodiscard]] auto solve(const rclcpp::Duration & period) const
    -> trajectory_msgs::msg::JointTrajectoryPoint override;

private:
  hierarchy::TaskHierarchy task_hierarchy_;
};

}  // namespace ik_solvers
