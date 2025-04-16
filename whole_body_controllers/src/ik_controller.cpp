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

#include "whole_body_controllers/ik_controller.hpp"

#include <Eigen/Dense>
#include <format>
#include <ranges>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace whole_body_controllers
{

namespace
{

auto reset_pose_msg(geometry_msgs::msg::Pose * msg) -> void
{
  msg->position.x = std::numeric_limits<double>::quiet_NaN();
  msg->position.y = std::numeric_limits<double>::quiet_NaN();
  msg->position.z = std::numeric_limits<double>::quiet_NaN();
  msg->orientation.x = std::numeric_limits<double>::quiet_NaN();
  msg->orientation.y = std::numeric_limits<double>::quiet_NaN();
  msg->orientation.z = std::numeric_limits<double>::quiet_NaN();
  msg->orientation.w = std::numeric_limits<double>::quiet_NaN();
}

auto reset_pose_msg(geometry_msgs::msg::PoseStamped * msg) -> void
{
  reset_pose_msg(&msg->pose);
  msg->header.frame_id.clear();
  msg->header.stamp = rclcpp::Time();
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
  param_listener_ = std::make_unique<ik_controller::ParamListener>(get_node());
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

  // TODO(evan-palmer): simplify this by using all joints except the locked joints??

  manipulator_dofs_ = params_.manipulator_joints;
  n_manipulator_dofs_ = manipulator_dofs_.size();

  dofs_.reserve(free_flyer_dofs_.size() + manipulator_dofs_.size());
  std::ranges::copy(free_flyer_dofs_, std::back_inserter(dofs_));
  std::ranges::copy(manipulator_dofs_, std::back_inserter(dofs_));
  n_dofs_ = dofs_.size();

  vel_dofs_.reserve(free_flyer_vel_dofs_.size() + manipulator_dofs_.size());
  std::ranges::copy(free_flyer_vel_dofs_, std::back_inserter(vel_dofs_));
  std::ranges::copy(manipulator_dofs_, std::back_inserter(vel_dofs_));
  n_vel_dofs_ = vel_dofs_.size();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  reference_.writeFromNonRT(geometry_msgs::msg::PoseStamped());

  command_interfaces_.reserve(n_dofs_ + n_vel_dofs_);
  system_state_values_.resize(n_dofs_, std::numeric_limits<double>::quiet_NaN());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
      reference_.writeFromNonRT(*msg);
    });

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "~/robot_description", qos, [this](const std::shared_ptr<std_msgs::msg::String> msg) {
      if (model_initialized_ || msg->data.empty()) {
        return;
      }

      // initialize pinocchio
      // we need to specify that the base is a free flyer joint
      model_ = std::make_shared<pinocchio::Model>();
      pinocchio::urdf::buildModelFromXML(msg->data, pinocchio::JointModelFreeFlyer(), *model_);

      // extract the locked joints from the parameters
      std::vector<std::string> locked_joints;
      std::vector<pinocchio::JointIndex> locked_joint_ids;
      std::ranges::transform(params_.locked_joints, std::back_inserter(locked_joint_ids), [this](const auto & joint) {
        return model_->getJointId(joint);
      });

      // build the reduced model
      pinocchio::Model reduced_model;
      pinocchio::buildReducedModel(*model_, locked_joint_ids, pinocchio::neutral(*model_), reduced_model);
      *model_ = reduced_model;

      data_ = std::make_shared<pinocchio::Data>(*model_);
      model_initialized_ = true;

      // initialize the ik solver
      ik_solver_loader_ = std::make_unique<pluginlib::ClassLoader<ik_solvers::IKSolver>>(
        "whole_body_controllers", "ik_solvers::IKSolver");
      ik_solver_ = ik_solver_loader_->createSharedInstance(params_.ik_solver);
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
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

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
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_dofs_);

  for (const auto & dof : dofs_) {
    config.names.push_back(std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION));
  }

  return config;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(free_flyer_dofs_.size(), std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(free_flyer_dofs_.size());

  for (size_t i = 0; i < free_flyer_dofs_.size(); ++i) {
    const auto & dof = free_flyer_dofs_[i];
    interfaces.emplace_back(
      get_node()->get_name(), std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION), &reference_interfaces_[i]);
  }

  return interfaces;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::transform_goal(const geometry_msgs::msg::PoseStamped & goal, const std::string & target_frame) const
  -> geometry_msgs::msg::PoseStamped
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  const auto transform = tf_buffer_->lookupTransform(target_frame, goal.header.frame_id, tf2::TimePointZero);
  tf2::doTransform(goal.pose, transformed_pose.pose, transform);
  transformed_pose.header.frame_id = target_frame;
  transformed_pose.header.stamp = goal.header.stamp;
  return transformed_pose;
}

auto IKController::update_reference_from_subscribers(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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
      transformed_pose = transform_goal(*current_reference, target_frame);
    }
    catch (const tf2::TransformException & ex) {
      return controller_interface::return_type::ERROR;
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

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::update_system_state_values() -> controller_interface::return_type
{
  for (std::size_t i = 0; i < system_state_values_.size(); ++i) {
    const auto out = state_interfaces_[i].get_optional();
    if (!out.has_value()) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to get state value for joint %s", dofs_[i].c_str());
      return controller_interface::return_type::ERROR;
    }
    system_state_values_[i] = out.value();
  }

  return controller_interface::return_type::OK;
}

auto IKController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  update_system_state_values();

  if (!model_initialized_) {
    RCLCPP_WARN(get_node()->get_logger(), "Waiting for the robot description to be published...");
    return controller_interface::return_type::OK;
  }

  const Eigen::VectorXd q = Eigen::VectorXd::Map(system_state_values_.data(), system_state_values_.size());
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
    if (!command_interfaces_[i].set_value(point.positions[i])) {
      RCLCPP_WARN(
        get_node()->get_logger(), std::format("Failed to set position command value for joint {}", dofs_[i]).c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  for (std::size_t i = 0; i < n_vel_dofs_; ++i) {
    if (!command_interfaces_[n_dofs_ + i].set_value(point.velocities[i])) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        std::format("Failed to set velocity command value for joint {}", vel_dofs_[i]).c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  // TODO(evan-palmer): publish controller state

  return controller_interface::return_type::OK;
}

}  // namespace whole_body_controllers

PLUGINLIB_EXPORT_CLASS(whole_body_controllers::IKController, controller_interface::ChainableControllerInterface)
