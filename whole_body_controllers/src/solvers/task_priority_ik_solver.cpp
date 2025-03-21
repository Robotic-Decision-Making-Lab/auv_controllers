#include "task_priority_ik_solver.hpp"

#include <vector>

namespace ik_solvers
{

namespace hierarchy
{

namespace
{

auto active_tasks(const ConstraintSet & task_set) -> ConstraintSet
{
  ConstraintSet active_task_set;
  for (const auto & constraint : task_set) {
    auto set_task = std::dynamic_pointer_cast<SetConstraint>(constraint);
    if (!set_task || set_task->is_active()) {
      active_task_set.insert(constraint);
    }
  }
  return active_task_set;
}

}  // namespace

auto TaskHierarchy::insert(const std::shared_ptr<Constraint> & constraint) -> void { constraints_.insert(constraint); }

auto TaskHierarchy::clear() -> void { constraints_.clear(); }

auto TaskHierarchy::set_constraints() const -> ConstraintSet
{
  ConstraintSet set_constraints;
  for (const auto & constraint : constraints_) {
    if (std::dynamic_pointer_cast<SetConstraint>(constraint)) {
      set_constraints.insert(constraint);
    }
  }
  return set_constraints;
}

auto TaskHierarchy::equality_constraints() const -> ConstraintSet
{
  ConstraintSet equality_constraints;
  for (const auto & constraint : constraints_) {
    if (!std::dynamic_pointer_cast<SetConstraint>(constraint)) {
      equality_constraints.insert(constraint);
    }
  }
  return equality_constraints;
}

auto TaskHierarchy::hierarchies() const -> std::vector<ConstraintSet>
{
  auto active = active_tasks(constraints_);
  const ConstraintSet equality_constraints = this->equality_constraints();
  const ConstraintSet set_constraints = this->set_constraints();

  // Return the equality tasks if there are no set constraints
  if (set_constraints.empty()) {
    return {active};
  }

  std::vector<ConstraintSet> result;

  // Next, generate the power set of all possible set constraints, exluding the empty set.
  const std::vector<std::shared_ptr<Constraint>> set_constraints_vector(set_constraints.begin(), set_constraints.end());
  const size_t n_set_constraints = set_constraints_vector.size();

  for (size_t mask = 1; mask < (1 << n_set_constraints); ++mask) {  // start from 1 to avoid the empty set
    ConstraintSet subset;

    // Use the bitmask to select subsets of set_constraints
    for (size_t i = 0; i < n_set_constraints; ++i) {
      if (mask & (1 << i)) {
        subset.insert(set_constraints_vector[i]);
      }
    }

    // Insert equality constraints into this subset
    for (const auto & eq_constraint : equality_constraints) {
      subset.insert(eq_constraint);
    }

    result.push_back(subset);
  }

  return result;
}

}  // namespace hierarchy

namespace
{

auto compute_jacobian_nullspace(const Eigen::MatrixXd & aug_jacobian) -> Eigen::MatrixXd
{
  const Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(aug_jacobian.cols(), aug_jacobian.cols());
  return eye - (aug_jacobian.completeOrthogonalDecomposition().pseudoInverse() * aug_jacobian);
}

auto compute_aug_jacobian(const std::vector<Eigen::MatrixXd> & jacobians) -> Eigen::MatrixXd
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

auto solve_hierarchy(
  hierarchy::ConstraintSet constraints,
  const Eigen::VectorXd & vel,
  std::vector<Eigen::MatrixXd> jacobians,
  Eigen::MatrixXd nullspace) -> Eigen::VectorXd
{
  if (constraints.empty()) {
    return vel;
  }

  const auto task = *constraints.begin();
  Eigen::MatrixXd x = task->jacobian() * nullspace;
  Eigen::VectorXd V_next =
    x.completeOrthogonalDecomposition().pseudoInverse() * (task->gain() * task->error() - task->jacobian() * vel);

  jacobians.push_back(task->jacobian());
  nullspace = compute_jacobian_nullspace(compute_aug_jacobian(jacobians));

  constraints.erase(constraints.begin());
  return solve_hierarchy(constraints, V_next, jacobians, nullspace);
}

auto solve_hierarchy(const hierarchy::ConstraintSet & task_set, const pinocchio::Model & model) -> Eigen::VectorXd
{
  if (task_set.empty()) {
    throw std::runtime_error("No constraints have been added to the task hierarchy.");
  }

  auto vel = Eigen::VectorXd::Zero(model.nv);
  std::vector<Eigen::MatrixXd> jacobians;
  auto nullspace = Eigen::MatrixXd::Identity(model.nv, model.nv);

  return solve_hierarchy(task_set, vel, jacobians, nullspace);
}

}  // namespace

auto TaskPriorityIKSolver::solve(rclcpp::Duration period) const -> trajectory_msgs::msg::JointTrajectoryPoint
{
  if (hierarchies.empty()) {
    throw std::runtime_error("No constraints have been added to the task hierarchy.");
  }

  const auto hierarchies = task_hierarchy_.hierarchies();
  Eigen::VectorXd solution;

  if (task_hierarchy_.set_constraints().empty()) {
    solution = solve_hierarchy(hierarchies.front(), model_);
  } else {
    std::vector<Eigen::VectorXd> solutions;

    for (const auto & hierarchy : hierarchies) {
      Eigen::VectorXd current_solution = solve_hierarchy(hierarchy, model_);
      auto set_constraints = task_hierarchy_.set_constraints();
      bool valid = true;
      for (const auto & set_constraint : set_constraints) {
        Eigen::MatrixXd proj = set_constraint->jacobian() * current_solution;

        // TODO(evan-palmer): Check whether or not the solution will violate the constraint
        // TODO(evan-palmer): check based on the activation threshold
        if (proj.maxCoeff() > set_constraint->upper_limit() || proj.minCoeff() < set_constraint->lower_limit()) {
          valid = false;
          break;
        }
        solutions.push_back(current_solution);
      }
    }
  }
}

}  // namespace ik_solvers
