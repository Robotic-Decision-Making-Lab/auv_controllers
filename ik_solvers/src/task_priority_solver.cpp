// Copyright 2025, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ik_solvers/task_priority_solver.hpp"

#include <algorithm>
#include <memory>
#include <ranges>
#include <vector>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pseudoinverse.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

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

/// Compute the error between two quaternions using eq. 2.12 in Gianluca Antonelli's Underwater Robotics book.
/// Note that we only need to minimize the vector part of the error.
auto quaternion_error(const Eigen::Quaterniond & q1, const Eigen::Quaterniond & q2) -> Eigen::Vector3d
{
  const Eigen::Vector3d q1_vec = q1.vec();
  const Eigen::Vector3d q2_vec = q2.vec();

  const double q1_w = q1.w();
  const double q2_w = q2.w();

  const Eigen::Vector3d vec_error = (q2_w * q1_vec) - (q1_w * q2_vec) + q2_vec.cross(q1_vec);

  // This is how we would compute the scalar error if we needed it
  // const double scalar_error = q1_w * q2_w + q1_vec.dot(q2_vec);

  return {vec_error.x(), vec_error.y(), vec_error.z()};
}

}  // namespace

auto TaskHierarchy::insert(const std::shared_ptr<Constraint> & constraint) -> void { constraints_.insert(constraint); }

auto TaskHierarchy::clear() -> void { constraints_.clear(); }

auto TaskHierarchy::set_constraints() const -> ConstraintSet  // NOLINT
{
  const ConstraintSet tasks = active_tasks(constraints_);
  ConstraintSet result;
  std::ranges::copy(
    tasks |
      std::views::filter([](const auto & task) { return std::dynamic_pointer_cast<SetConstraint>(task) != nullptr; }),
    std::inserter(result, result.end()));
  return result;
}

auto TaskHierarchy::equality_constraints() const -> ConstraintSet  // NOLINT
{
  const ConstraintSet tasks = active_tasks(constraints_);
  ConstraintSet result;
  std::ranges::copy(
    tasks | std::views::filter([](const auto & task) { return !std::dynamic_pointer_cast<SetConstraint>(task); }),
    std::inserter(result, result.end()));
  return result;
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
  const Eigen::Isometry3d & primal,
  const Eigen::Isometry3d & constraint,
  const std::string & frame,
  double gain,
  int priority)
: Constraint(primal.matrix(), constraint.matrix(), gain, priority)
{
  error_ = Eigen::VectorXd::Zero(6);
  error_.head<3>() = (constraint.translation() - primal.translation()).eval();
  error_.tail<3>() = quaternion_error(Eigen::Quaterniond(constraint.rotation()), Eigen::Quaterniond(primal.rotation()));
  jacobian_ = Eigen::MatrixXd::Zero(6, model->nv);
  pinocchio::getFrameJacobian(*model, *data, model->getFrameId(frame), pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);
}

JointConstraint::JointConstraint(
  const std::shared_ptr<pinocchio::Model> & model,
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

  // The Jacobian is a row vector with a 1 in the column corresponding to the joint
  jacobian_ = Eigen::MatrixXd::Zero(1, model->nv);
  jacobian_(0, model->joints[model->getJointId(joint_name)].idx_v()) = 1;
}

}  // namespace hierarchy

namespace
{

/// Compute the nullspace of the Jacobian matrix using the pseudoinverse.
auto compute_nullspace(const Eigen::MatrixXd & augmented_jacobian) -> Eigen::MatrixXd
{
  const Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(augmented_jacobian.cols(), augmented_jacobian.cols());
  return eye - (pinv::least_squares(augmented_jacobian) * augmented_jacobian);
}

/// Construct the augmented Jacobian matrix from a list of Jacobian matrices.
auto construct_augmented_jacobian(const std::vector<Eigen::MatrixXd> & jacobians) -> Eigen::MatrixXd
{
  if (jacobians.empty()) {
    throw std::invalid_argument("At least one Jacobian matrix must be provided.");
  }

  const int n_cols = jacobians.front().cols();
  const int n_rows = std::accumulate(
    jacobians.begin(), jacobians.end(), 0, [](int sum, const Eigen::MatrixXd & jac) { return sum + jac.rows(); });

  Eigen::MatrixXd augmented_jacobian(n_rows, n_cols);
  int current_row = 0;  // NOLINT(misc-const-correctness)

  for (const auto & jac : jacobians) {
    augmented_jacobian.block(current_row, 0, jac.rows(), jac.cols()) = jac;
    current_row += jac.rows();
  }

  return augmented_jacobian;
}

/// Closed-loop task priority IK.
auto tpik(const hierarchy::ConstraintSet & tasks, size_t nv, double damping)
  -> std::expected<Eigen::VectorXd, SolverError>
{
  if (tasks.empty()) {
    return std::unexpected(SolverError::NO_SOLUTION);
  }

  std::vector<Eigen::MatrixXd> jacobians;
  jacobians.reserve(tasks.size());

  Eigen::VectorXd vel = Eigen::VectorXd::Zero(nv);
  Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(nv, nv);

  for (const auto & task : tasks) {
    const Eigen::MatrixXd jacobian_inv = pinv::damped_least_squares(task->jacobian(), damping);
    vel += nullspace * jacobian_inv * (task->gain() * task->error());

    jacobians.push_back(task->jacobian());
    nullspace = compute_nullspace(construct_augmented_jacobian(jacobians));
  }

  return vel;
}

/// Check if the solution is feasible.
auto is_feasible(const hierarchy::ConstraintSet & constraints, const Eigen::VectorXd & solution) -> bool
{
  return std::ranges::all_of(constraints, [&solution](const auto & constraint) {
    auto set_task = std::dynamic_pointer_cast<hierarchy::SetConstraint>(constraint);
    const double pred = (constraint->jacobian() * solution).value();
    return (
      (set_task->primal() > set_task->lower_threshold() && pred < 0) ||
      (set_task->primal() < set_task->upper_threshold() && pred > 0));
  });
}

/// Search for the feasible solutions to the task hierarchy and return the one with the smallest norm.
auto search_solutions(
  const hierarchy::ConstraintSet & set_tasks,
  const std::vector<hierarchy::ConstraintSet> & hierarchies,
  size_t nv,
  double damping) -> std::expected<Eigen::VectorXd, SolverError>
{
  if (hierarchies.empty()) {
    return std::unexpected(SolverError::NO_SOLUTION);
  }

  // there is only one solution if there are no set tasks
  if (set_tasks.empty()) {
    return tpik(hierarchies.front(), nv, damping);
  }

  std::vector<Eigen::VectorXd> solutions;
  solutions.reserve(hierarchies.size());

  for (const auto & tasks : hierarchies) {
    const auto out = tpik(tasks, nv, damping);
    if (!out.has_value()) {
      continue;
    }

    const Eigen::VectorXd & current_solution = out.value();
    if (is_feasible(set_tasks, current_solution)) {
      solutions.push_back(current_solution);
    }
  }

  if (solutions.empty()) {
    return std::unexpected(SolverError::NO_SOLUTION);
  }

  // Choose the solution with the smallest norm
  return *std::ranges::min_element(solutions, {}, [](const auto & a) { return a.norm(); });
}

auto pinocchio_to_eigen(const pinocchio::SE3 & pose) -> Eigen::Isometry3d
{
  Eigen::Isometry3d result;
  result.translation() = pose.translation();
  result.linear() = pose.rotation();
  return result;
}

}  // namespace

auto TaskPriorityIKSolver::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::shared_ptr<pinocchio::Model> & model,
  const std::shared_ptr<pinocchio::Data> & data,
  const std::string & prefix) -> void
{
  IKSolver::initialize(node, model, data, prefix);
  param_listener_ = std::make_shared<task_priority_solver::ParamListener>(node_, prefix);
  params_ = param_listener_->get_params();
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto TaskPriorityIKSolver::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto TaskPriorityIKSolver::configure_parameters() -> void
{
  update_parameters();
  damping_ = params_.damping;
  ee_frame_ = params_.end_effector_frame_id;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto TaskPriorityIKSolver::solve_ik(const Eigen::Isometry3d & goal, const Eigen::VectorXd & q)
  -> std::expected<Eigen::VectorXd, SolverError>
{
  configure_parameters();

  hierarchy_.clear();

  // get the end effector pose as an Eigen Isometry3d
  const Eigen::Isometry3d ee_pose = pinocchio_to_eigen(data_->oMf[model_->getFrameId(ee_frame_)]);

  // insert the pose constraint
  const double gain = params_.end_effector_pose_task.gain;
  hierarchy_.insert(std::make_shared<hierarchy::PoseConstraint>(model_, data_, ee_pose, goal, ee_frame_, gain));

  for (const auto & joint_name : params_.constrained_joints) {
    // get the joint index in the configuration vector
    const int joint_id = model_->getJointId(joint_name);
    const int idx_q = model_->joints[joint_id].idx_q();

    // get the constraint parameters
    const double primal = q[idx_q];
    const double ub = model_->upperPositionLimit[idx_q];
    const double lb = model_->lowerPositionLimit[idx_q];
    const double tol = params_.joint_limit_task.constrained_joints_map[joint_name].safety_tolerance;
    const double activation = params_.joint_limit_task.constrained_joints_map[joint_name].activation_threshold;
    const double joint_limit_gain = params_.joint_limit_task.constrained_joints_map[joint_name].gain;

    // insert the joint limit constraint
    hierarchy_.insert(std::make_shared<hierarchy::JointConstraint>(
      model_, primal, ub, lb, tol, activation, joint_name, joint_limit_gain));
  }

  // find the safe solutions and choose the one with the smallest norm
  return search_solutions(hierarchy_.set_constraints(), hierarchy_.hierarchies(), model_->nv, damping_);
}

}  // namespace ik_solvers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ik_solvers::TaskPriorityIKSolver, ik_solvers::IKSolver)
