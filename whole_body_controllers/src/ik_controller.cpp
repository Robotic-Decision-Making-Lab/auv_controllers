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

auto to_eigen(const std::vector<double> & vec) -> Eigen::Affine3d
{
  if (vec.size() != 7) {
    throw std::invalid_argument("Invalid size for pose vector");
  }
  Eigen::Translation3d translation = {vec[0], vec[1], vec[2]};
  Eigen::Quaterniond rotation = {vec[6], vec[3], vec[4], vec[5]};
  rotation.normalize();
  return Eigen::Affine3d(translation * rotation);
}

}  // namespace

auto IKController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_unique<ik_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();
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

  // we do not require the manipulator joints to be provided in the same order as the pinocchio model
  // this will be handled in the state update
  manipulator_dofs_ = params_.manipulator_joints;
  n_manipulator_dofs_ = manipulator_dofs_.size();

  pos_dofs_.reserve(free_flyer_pos_dofs_.size() + manipulator_dofs_.size());
  std::ranges::copy(free_flyer_pos_dofs_, std::back_inserter(pos_dofs_));
  std::ranges::copy(manipulator_dofs_, std::back_inserter(pos_dofs_));
  n_pos_dofs_ = pos_dofs_.size();

  vel_dofs_.reserve(free_flyer_vel_dofs_.size() + manipulator_dofs_.size());
  std::ranges::copy(free_flyer_vel_dofs_, std::back_inserter(vel_dofs_));
  std::ranges::copy(manipulator_dofs_, std::back_inserter(vel_dofs_));
  n_vel_dofs_ = vel_dofs_.size();

  // keep track of which interfaces the solver will write to
  // we need to know these before we configure the command interfaces
  has_position_interface_ =
    std::ranges::find(params_.command_interfaces, "position") != params_.command_interfaces.end();
  has_velocity_interface_ =
    std::ranges::find(params_.command_interfaces, "velocity") != params_.command_interfaces.end();

  if (has_position_interface_) {
    n_command_interfaces_ += n_pos_dofs_;
  }
  if (has_velocity_interface_) {
    n_command_interfaces_ += n_vel_dofs_;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  // NOLINTBEGIN
  RCLCPP_INFO(logger_, "Commands won't be sent until both reference and state messages are received.");
  RCLCPP_INFO(logger_, "Waiting for robot description to be received");
  // NOLINTEND

  configure_parameters();

  reference_.writeFromNonRT(geometry_msgs::msg::Pose());
  vehicle_state_.writeFromNonRT(nav_msgs::msg::Odometry());

  command_interfaces_.reserve(n_command_interfaces_);
  system_state_values_.resize(n_pos_dofs_, std::numeric_limits<double>::quiet_NaN());

  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::Pose> msg) {
      // auv_controllers uses the maritime standard for all controllers, but pinocchio uses the ROS standard
      // convert to the ROS standard for *internal usage only*
      m2m::transforms::transform_message(*msg);
      reference_.writeFromNonRT(*msg);
    });

  if (params_.use_external_measured_vehicle_states) {
    RCLCPP_INFO(logger_, "Using external measured vehicle states");  // NOLINT
    vehicle_state_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      "~/vehicle_state", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        // TODO(evan-palmer): make this consistent with the vehicle state message
        // similar to the reference message, we need to transform the vehicle state message into the appropriate frame
        m2m::transforms::transform_message(*msg, "map", "base_link");
        vehicle_state_.writeFromNonRT(*msg);
      });
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "robot_description", qos, [this](const std::shared_ptr<std_msgs::msg::String> msg) {
      if (model_initialized_ || msg->data.empty()) {
        return;
      }

      // initialize pinocchio
      // we need to configure a free flyer joint for the base
      model_ = std::make_shared<pinocchio::Model>();
      pinocchio::urdf::buildModelFromXML(msg->data, pinocchio::JointModelFreeFlyer(), *model_);

      // extract the locked joints from the parameters
      // if we don't remove the locked joints, the Jacobian will be incorrect
      std::vector<std::string> locked_joints;
      std::vector<pinocchio::JointIndex> locked_joint_ids;
      locked_joints.reserve(params_.locked_joints.size());
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
      // we use the ik solver name as the prefix for its parameters
      // TODO(evan-palmer): look for memory leaks in solver
      loader_ = std::make_unique<pluginlib::ClassLoader<ik_solvers::IKSolver>>("ik_solvers", "ik_solvers::IKSolver");
      solver_ = loader_->createSharedInstance(params_.ik_solver);
      solver_->initialize(get_node(), model_, data_, params_.ik_solver);

      // NOLINTNEXTLINE
      RCLCPP_INFO(logger_, std::format("Initialized IK controller with solver {}", params_.ik_solver).c_str());
    });

  // TODO(evan-palmer): add controller state publisher

  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  common::messages::reset_message(reference_.readFromNonRT());
  common::messages::reset_message(vehicle_state_.readFromNonRT());
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto IKController::command_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_command_interfaces_);

  auto add_interfaces =
    [&config](const std::vector<std::string> & dofs, const std::string & reference, const std::string & interface) {
      for (const auto & dof : dofs) {
        config.names.push_back(
          reference.empty() ? std::format("{}/{}", dof, interface)
                            : std::format("{}/{}/{}", reference, dof, interface));
      }
    };

  // add the position interfaces
  if (has_position_interface_) {
    add_interfaces(free_flyer_pos_dofs_, params_.vehicle_reference_controller, hardware_interface::HW_IF_POSITION);
    add_interfaces(manipulator_dofs_, params_.manipulator_reference_controller, hardware_interface::HW_IF_POSITION);
  }

  // add the velocity interfaces
  if (has_velocity_interface_) {
    add_interfaces(free_flyer_vel_dofs_, params_.vehicle_reference_controller, hardware_interface::HW_IF_VELOCITY);
    add_interfaces(manipulator_dofs_, params_.manipulator_reference_controller, hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

auto IKController::state_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  if (params_.use_external_measured_vehicle_states) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(n_manipulator_dofs_);
    for (const auto & dof : manipulator_dofs_) {
      config.names.emplace_back(std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION));
    }
  } else {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(n_pos_dofs_);
    for (const auto & dof : pos_dofs_) {
      config.names.emplace_back(std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION));
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
    interfaces.emplace_back(
      get_node()->get_name(), std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION), &reference_interfaces_[i]);
  }

  return interfaces;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::update_reference_from_subscribers(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromRT();
  const auto reference = common::messages::to_vector(*current_reference);
  for (std::size_t i = 0; i < reference.size(); ++i) {
    if (!std::isnan(reference[i])) {
      reference_interfaces_[i] = reference[i];
    }
  }
  common::messages::reset_message(current_reference);
  return controller_interface::return_type::OK;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto IKController::update_system_state_values() -> controller_interface::return_type
{
  // we need the pinocchio model to be initialized before we can update the system state values
  // the pinocchio model is required to ensure that the state values are in the correct order
  if (!model_initialized_) {
    RCLCPP_INFO(logger_, "Waiting for the robot description to be published");  // NOLINT
    return controller_interface::return_type::ERROR;
  }

  // if we are using the external measured vehicle states, the message has already been transformed into the
  // appropriate frame, so we can just copy the values into the system state values. otherwise, we need to transform
  // the states first, then save them.
  const pinocchio::JointModel root = model_->joints[model_->getJointId("root_joint")];
  if (params_.use_external_measured_vehicle_states) {
    const auto * vehicle_state = vehicle_state_.readFromRT();
    std::ranges::copy(common::messages::to_vector(*vehicle_state), system_state_values_.begin() + root.idx_q());
  } else {
    std::vector<double> vehicle_states;
    vehicle_states.reserve(free_flyer_pos_dofs_.size());
    for (std::size_t i = 0; i < free_flyer_pos_dofs_.size(); ++i) {
      const auto out = state_interfaces_[i].get_optional();
      vehicle_states.push_back(out.value_or(std::numeric_limits<double>::quiet_NaN()));
    }

    geometry_msgs::msg::Pose pose;
    common::messages::to_msg(vehicle_states, &pose);
    m2m::transforms::transform_message(pose);
    std::ranges::copy(common::messages::to_vector(pose), system_state_values_.begin() + root.idx_q());
  }

  for (const auto & joint_name : manipulator_dofs_) {
    const pinocchio::JointModel joint = model_->joints[model_->getJointId(joint_name)];
    auto it = std::ranges::find_if(
      state_interfaces_, [joint_name](const auto & interface) { return interface.get_prefix_name() == joint_name; });

    if (it == state_interfaces_.end()) {
      RCLCPP_ERROR(logger_, "Could not find joint {} in state interfaces", joint_name);  // NOLINT
      return controller_interface::return_type::ERROR;
    }
    system_state_values_[joint.idx_q()] = it->get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }

  if (std::ranges::any_of(system_state_values_, [](double x) { return std::isnan(x); })) {
    RCLCPP_DEBUG(logger_, "Received system state with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }

  // save the vehicle state values
  // if (params_.use_external_measured_vehicle_states) {
  //   const auto * vehicle_state = vehicle_state_.readFromRT();
  //   std::ranges::copy(common::messages::to_vector(*vehicle_state), system_state_values_.begin());
  // } else {
  //   std::vector<double> states;
  //   states.reserve(free_flyer_pos_dofs_.size());

  //   for (const auto & [i, dof] : std::views::enumerate(free_flyer_pos_dofs_)) {
  //     const auto out = state_interfaces_[i].get_optional();
  //     states.push_back(out.value_or(std::numeric_limits<double>::quiet_NaN()));
  //   }

  //   // the system state interfaces use the maritime standard for states, so we need to convert the vehicle states
  //   // into the ROS standard before we save them for use with pinocchio
  //   geometry_msgs::msg::Pose pose;
  //   common::messages::to_msg(states, &pose);
  //   m2m::transforms::transform_message(pose);
  //   std::ranges::copy(common::messages::to_vector(pose), system_state_values_.begin());
  // }

  // // save the manipulator state values
  // for (std::size_t i = 0; i < n_manipulator_dofs_; ++i) {
  //   const std::size_t idx = params_.use_external_measured_vehicle_states ? i : free_flyer_pos_dofs_.size() + i;
  //   const auto out = state_interfaces_[idx].get_optional();
  //   system_state_values_[free_flyer_pos_dofs_.size() + i] = out.value_or(std::numeric_limits<double>::quiet_NaN());
  // }

  return controller_interface::return_type::OK;
}

auto IKController::update_and_validate_interfaces() -> controller_interface::return_type
{
  if (update_system_state_values() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Failed to update system state values");  // NOLINT
    return controller_interface::return_type::ERROR;
  }

  if (!model_initialized_) {
    RCLCPP_DEBUG(logger_, "Waiting for the robot description to be published");  // NOLINT
    return controller_interface::return_type::ERROR;
  }

  if (std::ranges::any_of(reference_interfaces_, [](double x) { return std::isnan(x); })) {
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

  const Eigen::VectorXd q = Eigen::VectorXd::Map(system_state_values_.data(), system_state_values_.size());
  const Eigen::Affine3d target_pose = to_eigen(reference_interfaces_);

  const auto result = solver_->solve(period, target_pose, q);

  if (!result.has_value()) {
    const auto err = result.error();
    if (err == ik_solvers::SolverError::NO_SOLUTION) {
      RCLCPP_WARN(logger_, "The solver could not find a solution to the current IK problem");
    } else if (err == ik_solvers::SolverError::SOLVER_ERROR) {
      RCLCPP_WARN(logger_, "The solver experienced an error while solving the IK problem");
    }
    return controller_interface::return_type::OK;
  }

  trajectory_msgs::msg::JointTrajectoryPoint point = result.value();

  // transform the solution into the appropriate frame
  geometry_msgs::msg::Twist twist;
  common::messages::to_msg({point.velocities.begin(), point.velocities.begin() + free_flyer_vel_dofs_.size()}, &twist);
  m2m::transforms::transform_message(twist);
  std::ranges::copy(common::messages::to_vector(twist), point.velocities.begin());

  // transform the pose into the appropriate frame
  geometry_msgs::msg::Pose pose;
  common::messages::to_msg({point.positions.begin(), point.positions.begin() + free_flyer_pos_dofs_.size()}, &pose);
  m2m::transforms::transform_message(pose);
  std::ranges::copy(common::messages::to_vector(pose), point.positions.begin());

  if (point.positions.size() != n_pos_dofs_) {
    RCLCPP_ERROR(
      logger_,
      std::format(
        "IK solution has mismatched position dimensions. Expected {} but got {}", n_pos_dofs_, point.positions.size())
        .c_str());
    return controller_interface::return_type::ERROR;
  }

  if (point.velocities.size() != n_vel_dofs_) {
    RCLCPP_ERROR(
      logger_,
      std::format(
        "IK solution has mismatched velocity dimensions. Expected {} but got {}", n_vel_dofs_, point.velocities.size())
        .c_str());
    return controller_interface::return_type::ERROR;
  }

  if (has_position_interface_) {
    for (std::size_t i = 0; i < n_pos_dofs_; ++i) {
      if (!command_interfaces_[i].set_value(point.positions[i])) {
        RCLCPP_WARN(  // NOLINT
          logger_,
          std::format("Failed to set position command value for joint {}", pos_dofs_[i]).c_str());
      }
    }
  }

  if (has_velocity_interface_) {
    for (std::size_t i = 0; i < n_vel_dofs_; ++i) {
      const std::size_t idx = has_position_interface_ ? n_pos_dofs_ + i : i;
      if (!command_interfaces_[idx].set_value(point.velocities[i])) {
        // NOLINTNEXTLINE
        RCLCPP_WARN(logger_, std::format("Failed to set velocity command value for joint {}", vel_dofs_[i]).c_str());
      }
    }
  }

  // TODO(evan-palmer): publish controller state

  return controller_interface::return_type::OK;
}

}  // namespace whole_body_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whole_body_controllers::IKController, controller_interface::ChainableControllerInterface)
