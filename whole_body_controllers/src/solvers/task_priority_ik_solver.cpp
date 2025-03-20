#include "task_priority_ik_solver.hpp"

#include <vector>

namespace ik_solvers
{

namespace
{

auto compute_jacobian_nullspace(const Eigen::MatrixXd & aug_jacobian) -> Eigen::MatrixXd
{
  const Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(aug_jacobian.cols(), aug_jacobian.cols());
  return eye - aug_jacobian.completeOrthogonalDecomposition().pseudoInverse() * aug_jacobian;
}

auto compose_aug_jacobian(const std::vector<Eigen::MatrixXd> & jacobians) -> Eigen::MatrixXd
{
  if (jacobians.empty()) {
    throw std::invalid_argument("At least one Jacobian matrix must be provided.");
  }

  if (jacobians.size() == 1) {
    return jacobians.front();
  }

  int n_rows = 0;
  const int n_cols = jacobians.front().cols();

  for (const auto & jac : jacobians) {
    n_rows += jac.rows();
    if (jac.cols() != n_cols) {
      throw std::invalid_argument("All Jacobian matrices must have the same number of columns.");
    }
  }

  Eigen::MatrixXd augmented(n_rows, n_cols);
  int current_row = 0;
  for (const auto & jac : jacobians) {
    augmented.block(current_row, 0, jac.rows(), jac.cols()) = jac;
    current_row += jac.rows();
  }

  return augmented;
}

}  // namespace

namespace hierarchy
{

auto TaskHierarchy::insert_constraint(std::shared_ptr<Constraint> constraint) -> void
{
  constraints_.insert(constraint);
}

auto TaskHierarchy::clear_constraints() -> void { constraints_.clear(); }

auto TaskHierarchy::active_tasks() const -> std::vector<std::shared_ptr<Constraint>>
{
  std::vector<std::shared_ptr<Constraint>> active_tasks;
  for (const auto & constraint : constraints_) {
    if (auto set_task = std::dynamic_pointer_cast<SetConstraint>(constraint)) {
      if (set_task->is_active()) {
        active_tasks.push_back(constraint);
      }
    } else {
      active_tasks.push_back(constraint);
    }
  }
  return active_tasks;
}

auto TaskHierarchy::hierarchies() const -> std::vector<std::set<std::shared_ptr<Constraint>>>
{
  std::vector<std::vector<std::shared_ptr<Constraint>>> hierarchies;
  std::vector<std::shared_ptr<Constraint>> active_tasks = this->active_tasks();

  /// Generate the power set of the active tasks (excluding the empty set).
  const int n = active_tasks.size();
  for (int i = 1; i < (1 << n); ++i) {
    std::vector<std::shared_ptr<Constraint>> hierarchy;
    for (int j = 0; j < n; ++j) {
      if (i & (1 << j)) {
        hierarchy.push_back(active_tasks[j]);
      }
    }
    hierarchies.push_back(hierarchy);
  }

  // TODO: use the comparator to sort the hierarchies
  return hierarchies;
}

}  // namespace hierarchy

auto TaskPriorityIKSolver::solve() const -> trajectory_msgs::msg::JointTrajectoryPoint {}

}  // namespace ik_solvers
