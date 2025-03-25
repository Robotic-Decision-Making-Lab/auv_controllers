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

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

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

auto reset_twist_msg(geometry_msgs::msg::Twist * msg) -> void
{
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}

auto reset_wrench_msg(geometry_msgs::msg::Wrench * msg) -> void
{
  msg->force.x = std::numeric_limits<double>::quiet_NaN();
  msg->force.y = std::numeric_limits<double>::quiet_NaN();
  msg->force.z = std::numeric_limits<double>::quiet_NaN();
  msg->torque.x = std::numeric_limits<double>::quiet_NaN();
  msg->torque.y = std::numeric_limits<double>::quiet_NaN();
  msg->torque.z = std::numeric_limits<double>::quiet_NaN();
}

auto reset_uvms_state_msg(auv_control_msgs::msg::UvmsState * msg) -> void
{
  reset_pose_msg(&msg->pose);
  reset_twist_msg(&msg->twist);
  reset_wrench_msg(&msg->wrench);
  msg->manipulator_joints.clear();
  msg->manipulator_positions.clear();
  msg->manipulator_velocities.clear();
  msg->manipulator_efforts.clear()
}

}  // namespace

auto IKController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_unique<ik_controller::ParamListener>(node_);
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::command_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // configure the end effector pose command interface
  for (const auto & dof : free_flyer_dofs_) {
    if (params_.reference_controller.empty()) {
      command_interface_configuration.names.emplace_back(
        params_.end_effector_frame_id + "/" + dof + "/" + hardware_interface::HW_IF_POSITION);
    } else {
      command_interface_configuration.names.emplace_back(
        params_.reference_controller + "/" + params_.end_effector_frame_id + "/" + dof + "/" +
        hardware_interface::HW_IF_POSITION);
    }
  }

  return command_interface_configuration;
}

auto IKController::state_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration state_interface_configuration;

  if (params_.use_external_measured_states) {
    state_interface_configuration.type = controller_interface::interface_configuration_type::NONE;
  } else {
    state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // add the vehicle pose state interface
    for (const auto & name : free_flyer_dofs_) {
      state_interface_configuration.names.emplace_back(
        params_.base_frame_id + "/" + name + "/" + hardware_interface::HW_IF_POSITION);
    }

    // add the manipulator joint state interface
    for (const auto & name : params_.manipulator_joints) {
      state_interface_configuration.names.emplace_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
  }

  return state_interface_configuration;
}

auto IKController::on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  // TODO(evan-palmer): implement
}

auto IKController::on_configure(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn
{
  update_parameters();

  reference_.writeFromNonRT(geometry_msgs::msg::PoseStamped());
  system_state_.writeFromNonRT(auv_control_msgs::msg::UvmsState());

  command_interfaces_.reserve(COMMAND_DOF);

  system_state_values_.resize(
    COMMAND_DOF + params_.manipulator_joints.size(), std::numeric_limits<double>::quiet_NaN());

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
      robot_description_ = msg->data;
      model_ = std::make_shared<pinocchio::Model>();
      pinocchio::urdf::buildModelFromXML(robot_description_, *model_, pinocchio::JointModelFreeFlyer());
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

auto IKController::on_activate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn
{
  reset_pose_msg(reference_.readFromNonRT());
  reset_uvms_state_msg(system_state_.readFromNonRT());

  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromRT();

  const std::vector<double> reference = {
    current_reference->position.x,
    current_reference->position.y,
    current_reference->position.z,
    current_reference->orientation.x,
    current_reference->orientation.y,
    current_reference->orientation.z,
    current_reference->orientation.w};

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

    const std::vector<double> state = {
      current_system_state->pose.position.x,
      current_system_state->pose.position.y,
      current_system_state->pose.position.z,
      current_system_state->pose.orientation.x,
      current_system_state->pose.orientation.y,
      current_system_state->pose.orientation.z,
      current_system_state->pose.orientation.w};

    for (std::size_t i = 0; i < params_.manipulator_joints.size(); ++i) {
      state.push_back(current_system_state->manipulator_positions[i]);
    }

    for (std::size_t i = 0; i < state.size(); ++i) {
      if (!std::isnan(state[i])) {
        system_state_values_[i] = state[i];
      }
    }

    reset_uvms_state_msg(current_system_state);
  } else {
    for (std::size_t i = 0; i < system_state_values_.size(); ++i) {
      system_state_values_[i] = state_interfaces_[i].get_value();
    }
  }

  return controller_interface::return_type::OK;
}

auto IKController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
}

auto IKController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto IKController::configure_parameters() -> void
{
  update_parameters();
  // TODO(evan-palmer): get parameter values
}

}  // namespace whole_body_controllers
