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

#include "controller_common/common.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "message_transforms/transforms.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace whole_body_controllers
{

namespace
{

auto to_eigen(const std::vector<double> & vec) -> Eigen::Isometry3d
{
  if (vec.size() != 7) {
    throw std::invalid_argument("Invalid size for pose vector");
  }
  const Eigen::Translation3d translation = {vec[0], vec[1], vec[2]};
  Eigen::Quaterniond rotation = {vec[6], vec[3], vec[4], vec[5]};
  rotation.normalize();
  return {translation * rotation};
}

}  // namespace

auto IKController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_unique<ik_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();

  // store the joint names
  free_flyer_pos_dofs_ = params_.free_flyer_position_joints;
  free_flyer_vel_dofs_ = params_.free_flyer_velocity_joints;

  // initialize the pinocchio model
  model_ = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModelFromXML(get_robot_description(), pinocchio::JointModelFreeFlyer(), *model_);

  // lock all uncontrolled joints
  std::vector<std::string> controlled_joints = {"universe", "root_joint"};
  std::ranges::copy(params_.controlled_joints, std::back_inserter(controlled_joints));

  std::vector<std::string> locked_joint_names;
  std::ranges::copy_if(model_->names, std::back_inserter(locked_joint_names), [&controlled_joints](const auto & name) {
    return std::ranges::find(controlled_joints, name) == controlled_joints.end();
  });

  std::vector<pinocchio::JointIndex> locked_joints;
  std::ranges::transform(locked_joint_names, std::back_inserter(locked_joints), [this](const auto & name) {
    return model_->getJointId(name);
  });

  // build the reduced model
  pinocchio::Model reduced_model;
  pinocchio::buildReducedModel(*model_, locked_joints, pinocchio::neutral(*model_), reduced_model);
  *model_ = reduced_model;
  data_ = std::make_shared<pinocchio::Data>(*model_);

  position_interface_names_.resize(model_->nq);
  velocity_interface_names_.resize(model_->nv);

  // save the interface names in the same order as the model
  // start at index 1 to skip the universe joint
  for (std::size_t i = 1; i < model_->names.size(); ++i) {
    const std::string name = model_->names[i];
    const auto joint = model_->joints[model_->getJointId(name)];
    if (name == "root_joint") {
      std::ranges::copy(free_flyer_pos_dofs_, position_interface_names_.begin() + joint.idx_q());
      std::ranges::copy(free_flyer_vel_dofs_, velocity_interface_names_.begin() + joint.idx_v());
    } else {
      position_interface_names_[joint.idx_q()] = name;
      velocity_interface_names_[joint.idx_v()] = name;
    }
  }

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

  auto has_interface = [](const std::vector<std::string> & interfaces, const std::string & type) -> bool {
    return std::ranges::find(interfaces, type) != interfaces.end();
  };

  use_position_commands_ = has_interface(params_.command_interfaces, "position");
  use_velocity_commands_ = has_interface(params_.command_interfaces, "velocity");

  n_command_interfaces_ = 0;
  if (use_position_commands_) {
    n_command_interfaces_ += position_interface_names_.size();
  }
  if (use_velocity_commands_) {
    n_command_interfaces_ += velocity_interface_names_.size();
  }

  use_position_states_ = has_interface(params_.state_interfaces, "position");
  use_velocity_states_ = has_interface(params_.state_interfaces, "velocity");

  n_state_interfaces_ = 0;
  if (use_position_states_) {
    n_state_interfaces_ += position_interface_names_.size();
  }
  if (use_velocity_states_) {
    n_state_interfaces_ += velocity_interface_names_.size();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  reference_.writeFromNonRT(geometry_msgs::msg::Pose());
  vehicle_state_.writeFromNonRT(nav_msgs::msg::Odometry());

  command_interfaces_.reserve(n_command_interfaces_);
  position_state_values_.resize(position_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());
  velocity_state_values_.resize(velocity_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::Pose> msg) {  // NOLINT
      m2m::transform_message(*msg);
      reference_.writeFromNonRT(*msg);
    });

  if (params_.use_external_measured_vehicle_states) {
    RCLCPP_INFO(logger_, "Using external measured vehicle states");  // NOLINT
    vehicle_state_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      "~/vehicle_state",
      rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {  // NOLINT
        m2m::transform_message(*msg, "map", "base_link");
        vehicle_state_.writeFromNonRT(*msg);
      });
  }

  loader_ = std::make_unique<pluginlib::ClassLoader<ik_solvers::IKSolver>>("ik_solvers", "ik_solvers::IKSolver");
  solver_ = loader_->createSharedInstance(params_.ik_solver);
  solver_->initialize(get_node(), model_, data_, params_.ik_solver);
  RCLCPP_INFO(logger_, "Configured the IK controller with solver %s", params_.ik_solver.c_str());  // NOLINT

  controller_state_pub_ = get_node()->create_publisher<auv_control_msgs::msg::IKControllerStateStamped>(
    "~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<auv_control_msgs::msg::IKControllerStateStamped>>(
      controller_state_pub_);

  rt_controller_state_pub_->lock();
  rt_controller_state_pub_->msg_.solver_name = params_.ik_solver;
  std::ranges::copy(position_interface_names_, std::back_inserter(rt_controller_state_pub_->msg_.position_joint_names));
  std::ranges::copy(velocity_interface_names_, std::back_inserter(rt_controller_state_pub_->msg_.velocity_joint_names));
  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  common::messages::reset_message(reference_.readFromNonRT());
  common::messages::reset_message(vehicle_state_.readFromNonRT());
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  position_state_values_.assign(position_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  velocity_state_values_.assign(velocity_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::command_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_command_interfaces_);

  auto format_interface = [](const std::string & name, const std::string & type, const std::string & reference) {
    return reference.empty() ? std::format("{}/{}", name, type) : std::format("{}/{}/{}", reference, name, type);
  };

  if (use_position_commands_) {
    std::ranges::transform(
      position_interface_names_, std::back_inserter(config.names), [this, &format_interface](const auto & name) {
        if (std::ranges::find(free_flyer_pos_dofs_, name) != free_flyer_pos_dofs_.end()) {
          return format_interface(name, hardware_interface::HW_IF_POSITION, params_.vehicle_reference_controller);
        }
        return format_interface(name, hardware_interface::HW_IF_POSITION, params_.manipulator_reference_controller);
      });
  }

  if (use_velocity_commands_) {
    std::ranges::transform(
      velocity_interface_names_, std::back_inserter(config.names), [this, &format_interface](const auto & name) {
        if (std::ranges::find(free_flyer_vel_dofs_, name) != free_flyer_vel_dofs_.end()) {
          return format_interface(name, hardware_interface::HW_IF_VELOCITY, params_.vehicle_reference_controller);
        }
        return format_interface(name, hardware_interface::HW_IF_VELOCITY, params_.manipulator_reference_controller);
      });
  }

  return config;
}

auto IKController::state_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  auto insert_interfaces = [&config](const std::vector<std::string> & interface_names, const std::string & type) {
    std::ranges::transform(interface_names, std::back_inserter(config.names), [type](const auto & name) {
      return std::format("{}/{}", name, type);
    });
  };

  if (params_.use_external_measured_vehicle_states) {
    config.names.reserve(params_.controlled_joints.size());
    if (use_position_states_) {
      insert_interfaces(params_.controlled_joints, hardware_interface::HW_IF_POSITION);
    }
    if (use_velocity_states_) {
      insert_interfaces(params_.controlled_joints, hardware_interface::HW_IF_VELOCITY);
    }
  } else {
    config.names.reserve(n_state_interfaces_);
    if (use_position_states_) {
      insert_interfaces(position_interface_names_, hardware_interface::HW_IF_POSITION);
    }
    if (use_velocity_states_) {
      insert_interfaces(velocity_interface_names_, hardware_interface::HW_IF_VELOCITY);
    }
  }

  return config;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(free_flyer_pos_dofs_.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(free_flyer_pos_dofs_.size());

  for (const auto & [i, dof] : std::views::enumerate(free_flyer_pos_dofs_)) {
    const std::string interface = std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION);
    interfaces.emplace_back(get_node()->get_name(), interface, &reference_interfaces_[i]);
  }

  return interfaces;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::update_reference_from_subscribers(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromRT();
  std::vector<double> reference = common::messages::to_vector(*current_reference);
  for (auto && [interface, ref] : std::views::zip(reference_interfaces_, reference)) {
    interface = ref;
  }
  common::messages::reset_message(current_reference);
  return controller_interface::return_type::OK;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::update_system_state_values() -> controller_interface::return_type
{
  // if we are using the external measured vehicle states, the message has already been transformed into the
  // appropriate frame, so we can just copy the values into the system state values. otherwise, we need to transform
  // the states first, then save them.
  if (params_.use_external_measured_vehicle_states) {
    const auto * state_msg = vehicle_state_.readFromRT();
    const std::vector<double> state = common::messages::to_vector(*state_msg);
    std::ranges::copy(state.begin(), state.begin() + free_flyer_pos_dofs_.size(), position_state_values_.begin());
    std::ranges::copy(state.begin() + free_flyer_pos_dofs_.size(), state.end(), velocity_state_values_.begin());
  } else {
    auto save_states = [](const auto & interfaces, auto out) {
      std::ranges::transform(interfaces, out, [](const auto & interface) {
        return interface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
      });
    };

    // retrieve the vehicle position and velocity state interfaces
    const auto position_interfaces_end = state_interfaces_.begin() + free_flyer_pos_dofs_.size();
    const auto velocity_interfaces_start = position_interfaces_end + params_.controlled_joints.size();
    const auto velocity_interfaces_end = velocity_interfaces_start + free_flyer_vel_dofs_.size();

    const auto position_interfaces = std::span(state_interfaces_.begin(), position_interfaces_end);
    const auto velocity_interfaces = std::span(velocity_interfaces_start, velocity_interfaces_end);

    std::vector<double> position_states, velocity_states;  // NOLINT(readability-isolate-declaration)
    position_states.reserve(position_interfaces.size());
    velocity_states.reserve(velocity_interfaces.size());

    save_states(position_interfaces, std::back_inserter(position_states));
    save_states(velocity_interfaces, std::back_inserter(velocity_states));

    // transform the states into the appropriate frame and save them
    geometry_msgs::msg::Pose pose;
    common::messages::to_msg(position_states, &pose);
    m2m::transform_message(pose);
    std::ranges::copy(common::messages::to_vector(pose), position_state_values_.begin());

    geometry_msgs::msg::Twist twist;
    common::messages::to_msg(velocity_states, &twist);
    m2m::transform_message(twist);
    std::ranges::copy(common::messages::to_vector(twist), velocity_state_values_.begin());
  }

  auto find_interface = [](const auto & interfaces, const std::string & name, const std::string & type) {
    return std::ranges::find_if(interfaces, [&name, &type](const auto & interface) {
      return interface.get_name() == std::format("{}/{}", name, type);
    });
  };

  // save the manipulator states
  for (const auto & joint_name : params_.controlled_joints) {
    const pinocchio::JointModel joint = model_->joints[model_->getJointId(joint_name)];

    const auto pos_it = find_interface(state_interfaces_, joint_name, hardware_interface::HW_IF_POSITION);
    if (pos_it != state_interfaces_.end()) {
      const double pos = pos_it->get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
      position_state_values_[joint.idx_q()] = pos;
    }

    const auto vel_it = find_interface(state_interfaces_, joint_name, hardware_interface::HW_IF_VELOCITY);
    if (vel_it != state_interfaces_.end()) {
      const double vel = vel_it->get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
      velocity_state_values_[joint.idx_v()] = vel;
    }
  }

  if (use_position_states_) {
    if (common::math::has_nan(position_state_values_)) {
      RCLCPP_DEBUG(logger_, "Received position states with NaN value.");  // NOLINT
      return controller_interface::return_type::ERROR;
    }
  }

  if (use_velocity_states_) {
    if (common::math::has_nan(velocity_state_values_)) {
      RCLCPP_DEBUG(logger_, "Received velocity states with NaN value.");  // NOLINT
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

auto IKController::update_chained_reference_values() -> controller_interface::return_type
{
  // the reference interfaces in chained mode are parsed directly from the command interfaces
  //
  // because the command interfaces expect the commands to be in the maritime coordinate frame standard, we need
  // this extra method to transform the values into a frame suitable for pinocchio
  geometry_msgs::msg::Pose reference_transformed;
  common::messages::to_msg(reference_interfaces_, &reference_transformed);
  m2m::transform_message(reference_transformed);
  std::ranges::copy(common::messages::to_vector(reference_transformed), reference_interfaces_.begin());
  return controller_interface::return_type::OK;
}

auto IKController::update_and_validate_interfaces() -> controller_interface::return_type
{
  if (update_system_state_values() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Failed to update system state values");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  if (is_in_chained_mode()) {
    update_chained_reference_values();
  }
  if (common::math::has_nan(reference_interfaces_)) {
    RCLCPP_DEBUG(logger_, "Received reference with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  if (update_and_validate_interfaces() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. Failed to update and validate interfaces");  // NOLINT
    return controller_interface::return_type::OK;
  }

  configure_parameters();

  const Eigen::VectorXd q = Eigen::VectorXd::Map(position_state_values_.data(), position_state_values_.size());
  const Eigen::Isometry3d goal = to_eigen(reference_interfaces_);

  // TODO(anyone): add solver support for velocity states
  // right now we only use the positions for the solver
  const auto result = solver_->solve(period, goal, q);

  if (!result.has_value()) {
    const auto err = result.error();
    if (err == ik_solvers::SolverError::NO_SOLUTION) {
      RCLCPP_DEBUG(logger_, "The solver could not find a solution to the current IK problem");  // NOLINT
    } else if (err == ik_solvers::SolverError::SOLVER_ERROR) {
      RCLCPP_WARN(logger_, "The solver experienced an error while solving the IK problem");  // NOLINT
    }
    return controller_interface::return_type::OK;
  }

  trajectory_msgs::msg::JointTrajectoryPoint point = result.value();

  // transform the solution into the appropriate frame
  geometry_msgs::msg::Twist twist;
  common::messages::to_msg({point.velocities.begin(), point.velocities.begin() + free_flyer_vel_dofs_.size()}, &twist);
  m2m::transform_message(twist);
  std::ranges::copy(common::messages::to_vector(twist), point.velocities.begin());

  // transform the pose into the appropriate frame
  geometry_msgs::msg::Pose pose;
  common::messages::to_msg({point.positions.begin(), point.positions.begin() + free_flyer_pos_dofs_.size()}, &pose);
  m2m::transform_message(pose);
  std::ranges::copy(common::messages::to_vector(pose), point.positions.begin());

  if (use_position_commands_) {
    for (const auto & [i, joint_name] : std::views::enumerate(position_interface_names_)) {
      if (!command_interfaces_[i].set_value(point.positions[i])) {
        RCLCPP_WARN(logger_, "Failed to set position command value for joint %s", joint_name.c_str());  // NOLINT
      }
    }
  }

  if (use_velocity_commands_) {
    for (const auto & [i, joint_name] : std::views::enumerate(velocity_interface_names_)) {
      const std::size_t idx = use_position_commands_ ? position_interface_names_.size() + i : i;
      if (!command_interfaces_[idx].set_value(point.velocities[i])) {
        RCLCPP_WARN(logger_, "Failed to set velocity command value for joint %s", joint_name.c_str());  // NOLINT
      }
    }
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = get_node()->now();
    rt_controller_state_pub_->msg_.time_step = period.seconds();
    common::messages::to_msg(reference_interfaces_, &rt_controller_state_pub_->msg_.reference);
    rt_controller_state_pub_->msg_.solution = point;
    rt_controller_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace whole_body_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whole_body_controllers::IKController, controller_interface::ChainableControllerInterface)
