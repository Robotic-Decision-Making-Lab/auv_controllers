#include "task_priority_ik_solver.hpp"

#include <algorithm>
#include <memory>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ranges>
#include <tf2_eigen/tf2_eigen.hpp>
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

/// Filter the set of tasks to include only set constraints.
auto filter_set_constraints(const ConstraintSet & tasks) -> ConstraintSet
{
  ConstraintSet result;
  std::ranges::copy(
    tasks |
      std::views::filter([](const auto & task) { return std::dynamic_pointer_cast<SetConstraint>(task) != nullptr; }),
    std::inserter(result, result.end()));
  return result;
}

/// Filter the set of tasks to include only equality constraints.
auto filter_equality_constraints(const ConstraintSet & tasks) -> ConstraintSet
{
  ConstraintSet result;
  std::ranges::copy(
    tasks | std::views::filter([](const auto & task) { return !std::dynamic_pointer_cast<SetConstraint>(task); }),
    std::inserter(result, result.end()));
  return result;
}

/// Compute the error between two quaternions using eq. 2.12 in Gianluca Antonelli's Underwater Robotics book.
/// Note that we only need to minimize the scalar part of the error.
auto quaternion_error(const Eigen::Quaterniond & q1, const Eigen::Quaterniond & q2) -> Eigen::Vector3d
{
  const Eigen::Vector3d q1_vec = q1.vec();
  const Eigen::Vector3d q2_vec = q2.vec();

  const double q1_w = q1.w();
  const double q2_w = q2.w();

  const Eigen::Vector3d vec_error = q2_w * q1_vec - q1_w * q2_vec + q2_vec.cross(q1_vec);

  // This is how we would compute the scalar error if we needed it
  // const double scalar_error = q1_w * q2_w + q1_vec.dot(q2_vec);

  return {vec_error.x(), vec_error.y(), vec_error.z()};
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

PoseConstraint::PoseConstraint(
  const std::shared_ptr<pinocchio::Model> & model,
  const std::shared_ptr<pinocchio::Data> & data,
  const Eigen::Affine3d & primal,
  const Eigen::Affine3d & constraint,
  const std::string & frame,
  double gain,
  int priority)
: Constraint(primal.matrix(), constraint.matrix(), gain, priority)
{
  error_ = Eigen::VectorXd::Zero(6);
  error_.head<3>() = (constraint.translation() - primal.translation()).eval();
  error_.tail<3>() = quaternion_error(Eigen::Quaterniond(constraint.rotation()), Eigen::Quaterniond(primal.rotation()));

  pinocchio::computeJointJacobians(*model, *data);
  jacobian_ = pinocchio::getFrameJacobian(*model, *data, model->getFrameId(frame), pinocchio::LOCAL_WORLD_ALIGNED);
}

JointConstraint::JointConstraint(
  const std::shared_ptr<pinocchio::Model> & model,
  const std::shared_ptr<pinocchio::Data> & data,
  double primal,
  double ub,
  double lb,
  double tol,
  double activation,
  const std::string & joint_name,
  double gain,
  int priority)
: SetConstraint(primal, ub, lb, tol, activation, gain, priority)
{
  error_ = constraint_ - primal_;

  pinocchio::computeJointJacobians(*model, *data);
  jacobian_ = pinocchio::getJointJacobian(*model, *data, model->getJointId(joint_name), pinocchio::LOCAL_WORLD_ALIGNED);
}

}  // namespace hierarchy

namespace
{

/// Compute the nullspace of the Jacobian matrix using the pseudoinverse.
auto compute_jacobian_nullspace(const Eigen::MatrixXd & aug_jacobian) -> Eigen::MatrixXd
{
  // TODO(evan-palmer): do we need the damping here?
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

/// Closed-loop task priority IK using the damped pseudoinverse.
auto tpik(const hierarchy::ConstraintSet & tasks, size_t nv, double damping) -> Eigen::VectorXd
{
  if (tasks.empty()) {
    throw std::runtime_error("No constraints have been added to the task hierarchy.");
  }

  auto vel = Eigen::VectorXd::Zero(nv);
  std::vector<Eigen::MatrixXd> jacs;
  Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(nv, nv);

  for (const auto & task : tasks) {
    const Eigen::MatrixXd x = task->jacobian() * nullspace;
    const auto eye = Eigen::MatrixXd::Identity(x.rows(), x.rows());

    const Eigen::MatrixXd x_inv = x.transpose() * (x * x.transpose() + damping * eye).inverse();
    const Eigen::VectorXd vel = x_inv * (task->gain() * task->error() - task->jacobian() * vel);

    jacs.push_back(task->jacobian());
    nullspace = compute_jacobian_nullspace(compute_aug_jacobian(jacs));
  }

  return vel;
}

/// Search for the feasible solutions to the task hierarchy and return the one with the smallest norm.
auto search_solutions(
  const hierarchy::ConstraintSet & set_tasks,
  const std::vector<hierarchy::ConstraintSet> & hierarchies,
  size_t nv,
  double damping) -> std::expected<Eigen::VectorXd, SolverError>
{
  Eigen::VectorXd solution;

  if (set_tasks.empty()) {
    solution = tpik(hierarchies.front(), nv, damping);
  } else {
    std::vector<Eigen::VectorXd> solutions;
    solutions.reserve(hierarchies.size());

    for (const auto & tasks : hierarchies) {
      const Eigen::VectorXd current_solution = tpik(tasks, nv, damping);

      // Check if the solution violates any set constraints
      bool valid = true;
      for (const auto & constraint : set_tasks) {
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
      return std::unexpected(SolverError::NO_SOLUTION);
    }

    // Choose the solution with the smallest norm
    solution = *std::ranges::min_element(solutions, {}, [](const auto & a) { return a.norm(); });
  }

  return solution;
}

auto pinocchio_to_eigen(const pinocchio::SE3 & pose) -> Eigen::Affine3d
{
  Eigen::Affine3d result;
  result.translation() = pose.translation();
  result.linear() = pose.rotation();
  return result;
}

}  // namespace

auto TaskPriorityIKSolver::solve_ik(
  const rclcpp::Duration & period,
  const Eigen::Affine3d & target_pose,
  const Eigen::VectorXd & q) -> std::expected<Eigen::VectorXd, SolverError>
{
  hierarchy_.clear();

  // get the end effector pose as an Eigen Affine3d
  const Eigen::Affine3d ee_pose = pinocchio_to_eigen(data_->oMf[model_->getFrameId(ee_frame_)]);

  // insert the pose constraint
  // TODO(evan-palmer): make the gain a parameter
  hierarchy_.insert(std::make_shared<hierarchy::PoseConstraint>(model_, data_, ee_pose, target_pose, ee_frame_, 0.1));

  // TODO(evan-palmer): insert the set constraints

  // Find the safe solutions and choose the one with the smallest norm
  return search_solutions(hierarchy_.set_constraints(), hierarchy_.hierarchies(), model_->nv, damping_);
}

}  // namespace ik_solvers

PLUGINLIB_EXPORT_CLASS(ik_solvers::TaskPriorityIKSolver, ik_solvers::IKSolver)
