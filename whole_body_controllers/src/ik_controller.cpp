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

#include "ik_controller.hpp"

#include <Eigen/Dense>
#include <format>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

namespace whole_body_controllers
{

namespace
{

auto reset_pose_msg(geometry_msgs::msg::Pose * msg) -> void
{
  *msg = {
    .position =
      {.x = std::numeric_limits<double>::quiet_NaN(),
       .y = std::numeric_limits<double>::quiet_NaN(),
       .z = std::numeric_limits<double>::quiet_NaN()},
    .orientation = {
      .x = std::numeric_limits<double>::quiet_NaN(),
      .y = std::numeric_limits<double>::quiet_NaN(),
      .z = std::numeric_limits<double>::quiet_NaN(),
      .w = std::numeric_limits<double>::quiet_NaN()}};
}

auto reset_twist_msg(geometry_msgs::msg::Twist * msg) -> void
{
  *msg = {
    .linear =
      {.x = std::numeric_limits<double>::quiet_NaN(),
       .y = std::numeric_limits<double>::quiet_NaN(),
       .z = std::numeric_limits<double>::quiet_NaN()},
    .angular = {
      .x = std::numeric_limits<double>::quiet_NaN(),
      .y = std::numeric_limits<double>::quiet_NaN(),
      .z = std::numeric_limits<double>::quiet_NaN()}};
}

auto reset_wrench_msg(geometry_msgs::msg::Wrench * msg) -> void
{
  *msg = {
    .force =
      {.x = std::numeric_limits<double>::quiet_NaN(),
       .y = std::numeric_limits<double>::quiet_NaN(),
       .z = std::numeric_limits<double>::quiet_NaN()},
    .torque = {
      .x = std::numeric_limits<double>::quiet_NaN(),
      .y = std::numeric_limits<double>::quiet_NaN(),
      .z = std::numeric_limits<double>::quiet_NaN()}};
}

auto reset_uvms_state_msg(auv_control_msgs::msg::UvmsState * msg, const std::vector<std::string> & joint_names) -> void
{
  reset_pose_msg(&msg->pose);
  reset_twist_msg(&msg->twist);
  reset_wrench_msg(&msg->wrench);
  msg->manipulator_joints = joint_names;
  msg->manipulator_torques.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->manipulator_positions.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->manipulator_velocities.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
}

auto pose_msg_to_vector(const geometry_msgs::msg::Pose & pose) -> std::vector<double>
{
  return {
    pose.position.x,
    pose.position.y,
    pose.position.z,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w};
}

auto vector_to_eigen(const std::vector<double> & vec) -> Eigen::Affine3d
{
  Eigen::Translation3d translation = {vec[0], vec[1], vec[2]};
  Eigen::Quaterniond quat = {vec[6], vec[3], vec[4], vec[5]};
  quat.normalize();
  return Eigen::Affine3d(translation * quat);
}

}  // namespace

auto IKController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_unique<ik_controller::ParamListener>(node_);
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto IKController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();
  manipulator_dofs_ = params_.manipulator_joints;
  dofs_ = std::views::concat(free_flyer_dofs_, manipulator_dofs_);
  n_manipulator_dofs_ = manipulator_dofs_.size();
  n_dofs_ = dofs_.size();
  vel_dofs_ = std::views::concat(free_flyer_velocity_dofs_, manipulator_dofs_);
  n_vel_dofs_ = vel_dofs_.size();
  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  update_parameters();

  reference_.writeFromNonRT(geometry_msgs::msg::PoseStamped());
  system_state_.writeFromNonRT(auv_control_msgs::msg::UvmsState());

  command_interfaces_.reserve(free_flyer_dofs_.size());
  system_state_values_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  reference_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
      reference_.writeFromNonRT(*msg);
    });

  if (params_.use_external_measured_states) {
    system_state_sub_ = node_->create_subscription<auv_control_msgs::msg::UvmsState>(
      "~/system_state",
      rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<auv_control_msgs::msg::UvmsState> msg) { system_state_.writeFromNonRT(*msg); });
  }

  robot_description_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "~/robot_description", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<std_msgs::msg::String> msg) {
      if (model_initialized_ || msg->data.empty()) {
        return;
      }

      // initialize pinocchio
      model_ = std::make_shared<pinocchio::Model>();
      pinocchio::urdf::buildModelFromXML(msg->data, *model_, pinocchio::JointModelFreeFlyer());
      data_ = std::make_shared<pinocchio::Data>(*model_);
      model_initialized_ = true;

      // initialize the ik solver
      ik_solver_loader_ = std::make_unique<pluginlib::ClassLoader<ik_solvers::IKSolver>>(
        "whole_body_controllers", "ik_solvers::IKSolver");
      ik_solver_ = ik_solver_loader_->createUniqueInstance(params_.ik_solver);
      ik_solver_->initialize(get_node(), model_, data_);

      RCLCPP_INFO(get_node()->get_logger(), "Initialized IK controller with solver %s", params_.ik_solver.c_str());
    });

  // TODO(evan-palmer): add controller state publisher

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  reset_pose_msg(reference_.readFromNonRT());
  reset_uvms_state_msg(system_state_.readFromNonRT(), manipulator_dofs_);
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto IKController::command_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // the ik controller produces both the velocity solution and integrated positions
  config.names.reserve(n_dofs_ + n_vel_dofs_);

  for (const auto & dof : dofs_) {
    config.names.push_back(
      params_.reference_controller.empty()
        ? std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION)
        : std::format("{}/{}/{}", params_.reference_controller, dof, hardware_interface::HW_IF_POSITION));
  }

  for (const auto & dof : vel_dofs_) {
    config.names.push_back(
      params_.reference_controller.empty()
        ? std::format("{}/{}", dof, hardware_interface::HW_IF_VELOCITY)
        : std::format("{}/{}/{}", params_.reference_controller, dof, hardware_interface::HW_IF_VELOCITY));
  }

  return config;
}

auto IKController::state_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = params_.use_external_measured_states ? controller_interface::interface_configuration_type::NONE
                                                     : controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_dofs_);

  for (const auto & dof : dofs_) {
    config.names.push_back(std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION));
  }

  return config;
}

auto IKController::on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(free_flyer_dofs_.size(), std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(free_flyer_dofs_.size());

  for (const auto & dof : free_flyer_dofs_) {
    interfaces.emplace_back(
      get_node()->get_name(), std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION), &reference_interfaces_[i]);
  }

  return interfaces;
}

auto IKController::transform_target_pose(
  const geometry_msgs::msg::PoseStamped & target,
  const std::string & target_frame) const -> geometry_msgs::msg::PoseStamped
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  const auto transform = tf_buffer_->lookupTransform(target_frame, target.header.frame_id, target.header.stamp);
  tf2::doTransform(target.pose, transformed_pose.pose, transform);
  transformed_pose.header.frame_id = target_frame;
  transformed_pose.header.stamp = target.header.stamp;
  return transformed_pose;
}

auto IKController::update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromRT();
  const std::string target_frame = params_.world_frame_id;

  geometry_msgs::msg::PoseStamped transformed_pose;
  if (current_reference->header.frame_id.empty() || current_reference->header.frame_id == target_frame) {
    transformed_pose = *current_reference;
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Reference pose is in frame %s, attempting to transform to %s",
      current_reference->header.frame_id.c_str(),
      target_frame.c_str());

    try {
      transformed_pose = transform_target_pose(*current_reference, target_frame);
    }
    catch (const tf2::TransformException & ex) {
      controller_interface::return_type::ERROR;
    }
  }

  const auto reference = pose_msg_to_vector(transformed_pose.pose);
  for (std::size_t i = 0; i < reference.size(); ++i) {
    if (!std::isnan(reference[i])) {
      reference_interfaces_[i] = reference[i];
    }
  }

  reset_pose_msg(current_reference);

  return controller_interface::return_type::OK;
}

auto IKController::update_system_state_values() -> controller_interface::return_type
{
  if (params_.use_external_measured_states) {
    auto * current_system_state = system_state_.readFromRT();

    std::vector<double> state = pose_msg_to_vector(current_system_state->base_pose);
    std::ranges::copy(current_system_state->positions, std::back_inserter(state));

    for (std::size_t i = 0; i < state.size(); ++i) {
      if (!std::isnan(state[i])) {
        system_state_values_[i] = state[i];
      }
    }

    reset_uvms_state_msg(current_system_state, manipulator_dofs_);
  } else {
    for (std::size_t i = 0; i < system_state_values_.size(); ++i) {
      system_state_values_[i] = state_interfaces_[i].get_value();
    }
  }

  return controller_interface::return_type::OK;
}

auto IKController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  update_system_state_values();

  const Eigen::VectorXd q(system_state_values_.data());
  const Eigen::Affine3d target_pose = vector_to_eigen(reference_interfaces_);

  const auto result = ik_solver_->solve(period, target_pose, q);

  if (!result.has_value()) {
    const auto err = result.error();
    if (err == ik_solvers::SolverError::NO_SOLUTION) {
      RCLCPP_WARN(get_node()->get_logger(), "The solver could not find a solution to the current IK problem");
    } else if (err == ik_solvers::SolverError::SOLVER_ERROR) {
      RCLCPP_WARN(get_node()->get_logger(), "The solver experienced an error while solving the IK problem");
    }
    return controller_interface::return_type::ERROR;
  }

  const trajectory_msgs::msg::JointTrajectoryPoint point = result.value();

  for (std::size_t i = 0; i < n_dofs_; ++i) {
    command_interfaces_[i].set_value(point.positions[i]);
  }

  for (std::size_t i = 0; i < n_vel_dofs_; ++i) {
    command_interfaces_[n_dofs_ + i].set_value(point.velocities[i]);
  }

  // TODO(evan-palmer): publish controller state

  return controller_interface::return_type::OK;
}

}  // namespace whole_body_controllers
