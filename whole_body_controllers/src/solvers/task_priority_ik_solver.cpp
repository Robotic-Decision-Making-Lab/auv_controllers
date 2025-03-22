#include "task_priority_ik_solver.hpp"

#include <algorithm>
#include <memory>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ranges>
#include <vector>

namespace ik_solvers
{

namespace hierarchy
{

namespace
{

/// Return the set of active tasks in the given hierarchy.
auto active_tasks(const ConstraintSet & tasks) -> ConstraintSet
{
  ConstraintSet result;
  std::ranges::copy(
    tasks | std::views::filter([](const auto & task) {
      auto set_task = std::dynamic_pointer_cast<SetConstraint>(task);
      return !std::dynamic_pointer_cast<SetConstraint>(set_task) || set_task->is_active();
    }),
    std::inserter(result, result.end()));
  return result;
}

auto filter_set_constraints(const ConstraintSet & tasks) -> ConstraintSet
{
  ConstraintSet result;
  std::ranges::copy(
    tasks |
      std::views::filter([](const auto & task) { return std::dynamic_pointer_cast<SetConstraint>(task) != nullptr; }),
    std::inserter(result, result.end()));
  return result;
}

auto filter_equality_constraints(const ConstraintSet & tasks) -> ConstraintSet
{
  ConstraintSet result;
  std::ranges::copy(
    tasks | std::views::filter([](const auto & task) { return !std::dynamic_pointer_cast<SetConstraint>(task); }),
    std::inserter(result, result.end()));
  return result;
}

}  // namespace

auto TaskHierarchy::insert(const std::shared_ptr<Constraint> & constraint) -> void { constraints_.insert(constraint); }

auto TaskHierarchy::clear() -> void { constraints_.clear(); }

auto TaskHierarchy::set_constraints() const -> ConstraintSet
{
  return filter_set_constraints(active_tasks(constraints_));
}

auto TaskHierarchy::equality_constraints() const -> ConstraintSet
{
  return filter_equality_constraints(active_tasks(constraints_));
}

auto TaskHierarchy::hierarchies() const -> std::vector<ConstraintSet>
{
  const ConstraintSet equality_constraints = this->equality_constraints();
  const ConstraintSet set_constraints = this->set_constraints();

  // Return the equality tasks if there are no set constraints
  if (set_constraints.empty()) {
    return {equality_constraints};
  }

  // Generate the power set of the set constraints, excluding the empty set
  const std::vector<std::shared_ptr<Constraint>> set_constraints_vector(set_constraints.begin(), set_constraints.end());
  const size_t n_subsets = (1 << set_constraints.size()) - 1;

  std::vector<ConstraintSet> result;
  result.reserve(n_subsets);

  for (const size_t mask : std::views::iota(1U, 1U << set_constraints.size())) {
    ConstraintSet subset;
    for (size_t i = 0; i < set_constraints.size(); ++i) {
      if (((mask >> i) & 1U) != 0) {
        subset.insert(set_constraints_vector[i]);
      }
    }
    subset.insert(equality_constraints.begin(), equality_constraints.end());
    result.push_back(std::move(subset));
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

  const int n_cols = jacobians.front().cols();
  int n_rows = 0;

  for (const auto & jac : jacobians) {
    if (jac.cols() != n_cols) {
      throw std::invalid_argument("All Jacobian matrices must have the same number of columns.");
    }
    n_rows += jac.rows();
  }

  Eigen::MatrixXd augmented(n_rows, n_cols);
  int current_row = 0;
  for (const auto & jac : jacobians) {
    augmented.block(current_row, 0, jac.rows(), jac.cols()) = jac;
    current_row += jac.rows();
  }

  return augmented;
}

/// Closed-loop TPIK using the damped pseudoinverse.
auto cltpik(hierarchy::ConstraintSet task_set, const pinocchio::Model & model, double damping) -> Eigen::VectorXd
{
  if (task_set.empty()) {
    throw std::runtime_error("No constraints have been added to the task hierarchy.");
  }

  auto vel = Eigen::VectorXd::Zero(model.nv);
  std::vector<Eigen::MatrixXd> jacs;
  Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(model.nv, model.nv);

  for (const auto & task : task_set) {
    const Eigen::MatrixXd x = task->jacobian() * nullspace;
    const auto eye = Eigen::MatrixXd::Identity(x.rows(), x.rows());

    const Eigen::MatrixXd x_inv = x.transpose() * (x * x.transpose() + damping * eye).inverse();
    const Eigen::VectorXd vel = x_inv * (task->gain() * task->error() - task->jacobian() * vel);

    jacs.push_back(task->jacobian());
    nullspace = compute_jacobian_nullspace(compute_aug_jacobian(jacs));
  }

  return vel;
}

}  // namespace

auto TaskPriorityIKSolver::update_pinocchio(const Eigen::VectorXd & q) const -> void
{
  pinocchio::forwardKinematics(*model_, *data_, q);
  pinocchio::updateFramePlacements(*model_, *data_);
  pinocchio::computeJointJacobians(*model_, *data_);
}

auto TaskPriorityIKSolver::solve(
  const rclcpp::Duration & period,
  const Eigen::Affine3d & target_pose,
  const Eigen::VectorXd & q) const -> trajectory_msgs::msg::JointTrajectoryPoint
{
  // Update pinocchio data
  update_pinocchio(q);

  // TODO(evan-palmer): update the constraints with the current state

  // Get the power set of the task hierarchy
  const auto hierarchies = task_hierarchy_.hierarchies();

  if (hierarchies.empty()) {
    throw std::runtime_error("No constraints have been added to the task hierarchy.");
  }

  Eigen::VectorXd solution;
  const auto set_constraints = task_hierarchy_.set_constraints();

  if (set_constraints.empty()) {
    solution = cltpik(hierarchies.front(), *model_, damping_);
  } else {
    std::vector<Eigen::VectorXd> solutions;
    solutions.reserve(hierarchies.size());

    for (const auto & tasks : hierarchies) {
      const Eigen::VectorXd current_solution = cltpik(tasks, *model_, damping_);

      // Check if the solution violates any set constraints
      bool valid = true;
      for (const auto & constraint : set_constraints) {
        auto set_task = std::dynamic_pointer_cast<hierarchy::SetConstraint>(constraint);
        const double pred = (constraint->jacobian() * current_solution).value();
        const double primal = set_task->primal();

        if ((primal > set_task->lower_threshold() && pred < 0) || (primal < set_task->upper_threshold() && pred > 0)) {
          valid = false;
          break;
        }

        // TODO(evan-palmer): check whether or not this is needed
        // if (std::abs(pred) > 0.02) {
        //   valid = false;
        // }
      }

      if (valid) {
        solutions.push_back(current_solution);
      }
    }

    if (solutions.empty()) {
      throw std::runtime_error("No valid solutions found for the task hierarchy.");
    }

    // Choose the solution with the smallest norm
    solution = *std::ranges::min_element(solutions, {}, [](const auto & a) { return a.norm(); });
  }

  // Integrate the solution to get the new joint positions
  const Eigen::VectorXd q_next = pinocchio::integrate(*model_, q, period.seconds() * solution);

  // Convert the solution into a JointTrajectoryPoint
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = period;
  // TODO(evan-palmer): Fill in the joint positions and velocities

  return point;
}

}  // namespace ik_solvers
