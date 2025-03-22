#include "task_priority_ik_solver.hpp"

#include <vector>

namespace ik_solvers
{

namespace hierarchy
{

namespace
{

/// Return the set of active tasks in the given hierarchy.
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

  // Generate the power set of the set constraints, excluding the empty set
  const std::vector<std::shared_ptr<Constraint>> set_constraints_vector(set_constraints.begin(), set_constraints.end());
  const size_t n_set_constraints = set_constraints_vector.size();

  for (size_t mask = 1; mask < (1 << n_set_constraints); ++mask) {  // start from 1 to avoid the empty set
    ConstraintSet subset;

    for (size_t i = 0; i < n_set_constraints; ++i) {
      if ((mask & (1 << i)) != 0U) {
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

/// Compute the nullspace of the Jacobian matrix using the pseudoinverse.
auto compute_jacobian_nullspace(const Eigen::MatrixXd & aug_jacobian) -> Eigen::MatrixXd
{
  const Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(aug_jacobian.cols(), aug_jacobian.cols());
  return eye - (aug_jacobian.completeOrthogonalDecomposition().pseudoInverse() * aug_jacobian);
}

/// Construct the augmented Jacobian matrix from a list of Jacobian matrices.
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

/// Recursively solve the IK problem using the task hierarchy.
auto solve_hierarchy(
  hierarchy::ConstraintSet constraints,
  const Eigen::VectorXd & vel,
  std::vector<Eigen::MatrixXd> jacobians,
  Eigen::MatrixXd nullspace) -> Eigen::VectorXd
{
  // Base case: no constraints left to solve
  if (constraints.empty()) {
    return vel;
  }

  // Calculate the velocity for the current task
  const auto task = *constraints.begin();
  const Eigen::MatrixXd x = (task->jacobian() * nullspace).completeOrthogonalDecomposition().pseudoInverse();
  const Eigen::VectorXd vel_next = x * (task->gain() * task->error() - task->jacobian() * vel);

  // Recursively solve the remaining tasks
  jacobians.push_back(task->jacobian());
  nullspace = compute_jacobian_nullspace(compute_aug_jacobian(jacobians));
  constraints.erase(constraints.begin());

  return solve_hierarchy(constraints, vel_next, jacobians, nullspace);
}

/// Inverse kinematics using the task hierarchy.
auto solve_hierarchy(const hierarchy::ConstraintSet & task_set, const pinocchio::Model & model) -> Eigen::VectorXd
{
  if (task_set.empty()) {
    throw std::runtime_error("No constraints have been added to the task hierarchy.");
  }

  auto vel = Eigen::VectorXd::Zero(model.nv);
  const std::vector<Eigen::MatrixXd> jacobians;
  auto nullspace = Eigen::MatrixXd::Identity(model.nv, model.nv);

  return solve_hierarchy(task_set, vel, jacobians, nullspace);
}

}  // namespace

auto TaskPriorityIKSolver::solve(const rclcpp::Duration & /*period*/) const
  -> trajectory_msgs::msg::JointTrajectoryPoint
{
  const auto hierarchies = task_hierarchy_.hierarchies();

  if (hierarchies.empty()) {
    throw std::runtime_error("No constraints have been added to the task hierarchy.");
  }

  Eigen::VectorXd solution;

  if (task_hierarchy_.set_constraints().empty()) {
    solution = solve_hierarchy(hierarchies.front(), *model_);
  } else {
    std::vector<Eigen::VectorXd> solutions;

    for (const auto & hierarchy : hierarchies) {
      // Get the system velocities for the current hierarchy
      const Eigen::VectorXd current_solution = solve_hierarchy(hierarchy, *model_);

      // Check if the solution violates any set constraints
      auto set_constraints = task_hierarchy_.set_constraints();
      for (const auto & constraint : set_constraints) {
        auto set_task = std::dynamic_pointer_cast<hierarchy::SetConstraint>(constraint);
        const double pred = (constraint->jacobian() * current_solution).value();

        bool valid = true;
        const double primal = set_task->primal();
        if (primal > set_task->lower_threshold() && pred < 0) {
          valid = false;
        } else if (primal < set_task->upper_threshold() && pred > 0) {
          valid = false;
        }

        // TODO(evan-palmer): check whether or not this is needed
        // if (std::abs(pred) > 0.02) {
        //   valid = false;
        // }

        if (valid) {
          solutions.push_back(current_solution);
        }
      }
    }
    if (solutions.empty()) {
      throw std::runtime_error("No valid solutions found for the task hierarchy.");
    }

    // Choose the solution with the smallest norm
    solution = solutions.front();
    double min_norm = solution.norm();
    for (const auto & sol : solutions) {
      const double norm = sol.norm();
      if (norm < min_norm) {
        solution = sol;
        min_norm = norm;
      }
    }
  }

  // Convert the solution into a JointTrajectoryPoint
  trajectory_msgs::msg::JointTrajectoryPoint point;

  // TODO: Forward kinematics to get the joint positions from the solution
  // TODO: Fill in the joint positions and velocities

  return point;
}

}  // namespace ik_solvers
