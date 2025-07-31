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

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "controller_common/common.hpp"
#include "geometric_trajectory_controller/geometric_trajectory_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace geometric_controllers
{

namespace
{

auto geodesic_error(const geometry_msgs::msg::Pose & goal, const geometry_msgs::msg::Pose & state) -> double
{
  Eigen::Isometry3d goal_mat, state_mat;  // NOLINT
  tf2::fromMsg(goal, goal_mat);
  tf2::fromMsg(state, state_mat);
  return std::pow((goal_mat.inverse() * state_mat).matrix().log().norm(), 2);
}

}  // namespace

auto GeometricTrajectoryController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<geometric_trajectory_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();
  return controller_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto GeometricTrajectoryController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto GeometricTrajectoryController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();

  dofs_ = params_.joints;
  n_dofs_ = dofs_.size();

  default_path_tolerance_ = params_.path_tolerance;
  default_goal_tolerance_ = params_.goal_tolerance;

  rt_path_tolerance_.writeFromNonRT(params_.path_tolerance);
  rt_goal_tolerance_.writeFromNonRT(params_.goal_tolerance);

  auto period = std::chrono::duration<double>(1.0 / params_.action_monitor_rate);
  action_monitor_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(period);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto GeometricTrajectoryController::validate_trajectory(
  const auv_control_msgs::msg::GeometricTrajectory & trajectory) const -> bool
{
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(logger_, "Received empty trajectory");  // NOLINT
    return false;
  }

  for (const auto & point : trajectory.points) {
    if (common::math::has_nan(common::messages::to_vector(point.point))) {
      RCLCPP_ERROR(logger_, "Received trajectory point with NaN value");  // NOLINT
      return false;
    }
  }

  const rclcpp::Time start_time = trajectory.header.stamp;
  if (start_time.seconds() != 0.0) {
    const rclcpp::Time end_time = start_time + trajectory.points.back().time_from_start;
    if (end_time < get_node()->now()) {
      RCLCPP_ERROR(logger_, "Received trajectory with end time in the past");  // NOLINT
      return false;
    }
  }

  // NOLINTNEXTLINE(readability-use-anyofallof)
  for (const auto [p1, p2] : std::views::zip(trajectory.points, trajectory.points | std::views::drop(1))) {
    const rclcpp::Duration p1_start = p1.time_from_start;
    const rclcpp::Duration p2_start = p2.time_from_start;
    if (p1_start >= p2_start) {
      RCLCPP_ERROR(logger_, "Trajectory points are not in order");  // NOLINT
      return false;
    }
  }

  return true;
}

auto GeometricTrajectoryController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  state_.writeFromNonRT(geometry_msgs::msg::Pose());
  command_interfaces_.reserve(n_dofs_);
  update_period_ = rclcpp::Duration(0.0, static_cast<uint32_t>(1.0e9 / static_cast<double>(get_update_rate())));

  if (params_.use_external_measured_states) {
    state_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
      "~/state",
      rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Pose> msg) {  // NOLINT
        state_.writeFromNonRT(*msg);
      });
  } else {
    if (params_.lookup_transform) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }
  }

  trajectory_sub_ = get_node()->create_subscription<auv_control_msgs::msg::GeometricTrajectory>(
    "~/trajectory",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<auv_control_msgs::msg::GeometricTrajectory> msg) {  // NOLINT
      auto updated_msg = *msg;
      if (!validate_trajectory(updated_msg)) {
        RCLCPP_ERROR(logger_, "Ignoring invalid trajectory message");  // NOLINT
        return;
      }
      const rclcpp::Time start_time = updated_msg.header.stamp;
      if (common::math::isclose(start_time.seconds(), 0.0)) {
        updated_msg.header.stamp = get_node()->now();
      }
      RCLCPP_INFO(logger_, "Received new trajectory message");  // NOLINT
      rt_trajectory_.writeFromNonRT(Trajectory(updated_msg, *state_.readFromNonRT()));
      rt_goal_tolerance_.writeFromNonRT(default_goal_tolerance_);
      rt_path_tolerance_.writeFromNonRT(default_path_tolerance_);
      rt_first_sample_.writeFromNonRT(true);
      rt_holding_position_.writeFromNonRT(false);
    });

  auto handle_goal = [this](
                       const rclcpp_action::GoalUUID & /*uuid*/,
                       std::shared_ptr<const FollowTrajectory::Goal> goal) {  // NOLINT
    RCLCPP_INFO(logger_, "Received new trajectory goal");                     // NOLINT
    if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_ERROR(logger_, "Can't accept new action goals. Controller is not running.");  // NOLINT
      return rclcpp_action::GoalResponse::REJECT;
    }
    auto updated_msg = goal->trajectory;  // make a non-const copy
    if (!validate_trajectory(updated_msg)) {
      RCLCPP_ERROR(logger_, "Ignoring invalid trajectory message");  // NOLINT
      return rclcpp_action::GoalResponse::REJECT;
    }
    const rclcpp::Time start_time = updated_msg.header.stamp;
    if (common::math::isclose(start_time.seconds(), 0.0)) {
      updated_msg.header.stamp = get_node()->now();
    }
    rt_trajectory_.writeFromNonRT(Trajectory(updated_msg, *state_.readFromNonRT()));
    rt_first_sample_.writeFromNonRT(true);
    rt_holding_position_.writeFromNonRT(false);
    RCLCPP_INFO(logger_, "Accepted new trajectory goal");  // NOLINT
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>> gh) {  // NOLINT
    RCLCPP_INFO(logger_, "Received cancel action goal");                                                      // NOLINT
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal && active_goal->gh_ == gh) {
      RCLCPP_INFO(logger_, "Canceling active goal");  // NOLINT
      auto action_result = std::make_shared<FollowTrajectory::Result>();
      active_goal->setCanceled(action_result);
      rt_holding_position_.writeFromNonRT(true);
      rt_first_sample_.writeFromNonRT(true);
      rt_goal_in_progress_.writeFromNonRT(false);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>> gh) {  // NOLINT
    RCLCPP_INFO(logger_, "Received accepted action goal");                                                // NOLINT
    rt_goal_in_progress_.writeFromNonRT(true);
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal) {
      RCLCPP_INFO(logger_, "Canceling active goal");  // NOLINT
      auto action_result = std::make_shared<FollowTrajectory::Result>();
      action_result->error_code = FollowTrajectory::Result::INVALID_GOAL;
      action_result->error_string = "Current goal cancelled by a new incoming action.";
      active_goal->setCanceled(action_result);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }

    rt_goal_tolerance_.writeFromNonRT(gh->get_goal()->goal_tolerance);
    rt_path_tolerance_.writeFromNonRT(gh->get_goal()->path_tolerance);

    const RealtimeGoalHandlePtr rt_gh = std::make_shared<RealtimeGoalHandle>(gh);
    rt_gh->execute();
    rt_active_goal_.writeFromNonRT(rt_gh);

    goal_handle_timer_.reset();
    goal_handle_timer_ = get_node()->create_wall_timer(action_monitor_period_, [rt_gh]() { rt_gh->runNonRealtime(); });
  };

  action_server_ = rclcpp_action::create_server<auv_control_msgs::action::FollowGeometricTrajectory>(
    get_node(), "~/follow_trajectory", handle_goal, handle_cancel, handle_accepted);

  controller_state_pub_ = get_node()->create_publisher<ControllerState>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerState>>(controller_state_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto GeometricTrajectoryController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  rt_first_sample_.writeFromNonRT(true);
  rt_holding_position_.writeFromNonRT(true);  // hold position until a trajectory is received
  common::messages::reset_message(state_.readFromNonRT());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto GeometricTrajectoryController::command_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_dofs_);

  std::ranges::transform(dofs_, std::back_inserter(config.names), [this](const auto & dof) {
    return params_.reference_controller.empty()
             ? std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION)
             : std::format("{}/{}/{}", params_.reference_controller, dof, hardware_interface::HW_IF_POSITION);
  });

  return config;
}

auto GeometricTrajectoryController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  if (params_.use_external_measured_states || params_.lookup_transform) {
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(n_dofs_);

  std::ranges::transform(dofs_, std::back_inserter(config.names), [](const auto & dof) {
    return std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION);
  });

  return config;
}

auto GeometricTrajectoryController::update_system_state() -> controller_interface::return_type
{
  auto * system_state = state_.readFromRT();
  if (params_.lookup_transform) {
    const auto to_frame = params_.child_frame_id;
    const auto from_frame = params_.frame_id;
    try {
      const auto transform = tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
      system_state->position.x = transform.transform.translation.x;
      system_state->position.y = transform.transform.translation.y;
      system_state->position.z = transform.transform.translation.z;
      system_state->orientation = transform.transform.rotation;
    }
    catch (const tf2::TransformException & ex) {
      const auto err = std::format("Failed to get transform from {} to {}: {}", from_frame, to_frame, ex.what());
      RCLCPP_DEBUG(logger_, err.c_str());  // NOLINT
      return controller_interface::return_type::ERROR;
    }
  } else if (!params_.use_external_measured_states) {
    auto get_value = [](const auto & interface) -> double {
      return interface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    };
    system_state->position.x = get_value(state_interfaces_[0]);
    system_state->position.y = get_value(state_interfaces_[1]);
    system_state->position.z = get_value(state_interfaces_[2]);
    system_state->orientation.x = get_value(state_interfaces_[3]);
    system_state->orientation.y = get_value(state_interfaces_[4]);
    system_state->orientation.z = get_value(state_interfaces_[5]);
    system_state->orientation.w = get_value(state_interfaces_[6]);
  }

  if (common::math::has_nan(common::messages::to_vector(*system_state))) {
    RCLCPP_DEBUG(logger_, "Received state with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

auto GeometricTrajectoryController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  if (update_system_state() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. Failed to update and validate interfaces");  // NOLINT
    return controller_interface::return_type::OK;
  }

  const auto active_goal = *rt_active_goal_.readFromRT();
  const bool goal_in_progress = *rt_goal_in_progress_.readFromRT();
  if (goal_in_progress && active_goal == nullptr) {
    RCLCPP_DEBUG(logger_, "Goal in progress but no active goal. Ignoring update");  // NOLINT
    return controller_interface::return_type::OK;
  }

  const geometry_msgs::msg::Pose system_state = *state_.readFromRT();
  geometry_msgs::msg::Pose reference_state;
  common::messages::reset_message(&reference_state);
  auto command_state = system_state;
  double error = std::numeric_limits<double>::quiet_NaN();

  auto publish_controller_state = [this, &reference_state, &system_state, &error, &command_state]() {
    controller_state_.header.stamp = get_node()->now();
    controller_state_.reference = reference_state;
    controller_state_.feedback = system_state;
    controller_state_.error = error;
    controller_state_.output = command_state;
    rt_controller_state_pub_->try_publish(controller_state_);
  };

  // hold position until a new trajectory is received
  if (*rt_holding_position_.readFromRT()) {
    write_command(command_interfaces_, system_state);
    publish_controller_state();
    return controller_interface::return_type::OK;
  }

  // set the sample time
  rclcpp::Time sample_time = time;
  if (*rt_first_sample_.readFromRT()) {
    rt_trajectory_.readFromRT()->reset_initial_state(system_state);
    rt_first_sample_.writeFromNonRT(false);
    sample_time += period;
  }

  // we use the current sample to measure errors and the future sample as the command
  // the future sample should be used in order to prevent the controller from lagging
  const auto * trajectory = rt_trajectory_.readFromRT();
  const auto sampled_reference = trajectory->sample(sample_time);
  const auto sampled_command = trajectory->sample(sample_time + update_period_);

  // get the reference state and error
  // the scenarios where this will not have a value are when the reference time is before or after the trajectory
  if (sampled_reference.has_value()) {
    reference_state = sampled_reference.value();
    error = geodesic_error(reference_state, system_state);
  }

  bool path_tolerance_exceeded = false;
  bool goal_tolerance_exceeded = false;
  bool trajectory_suceeded = false;

  if (sampled_command.has_value()) {
    command_state = sampled_command.value();
    if (!std::isnan(error)) {
      const double path_tolerance = *rt_path_tolerance_.readFromRT();
      if (path_tolerance > 0.0 && error > path_tolerance) {
        path_tolerance_exceeded = true;
        rt_holding_position_.writeFromNonRT(true);
        command_state = system_state;
        RCLCPP_WARN(logger_, "Aborting trajectory. Error threshold exceeded during execution: %f", error);  // NOLINT
        RCLCPP_INFO(logger_, "Holding position until a new trajectory is received");                        // NOLINT
      }
    }
  } else {
    switch (sampled_command.error()) {
      case SampleError::SAMPLE_TIME_BEFORE_START:
        RCLCPP_INFO(logger_, "Trajectory sample time is before trajectory start time");  // NOLINT
        RCLCPP_INFO(logger_, "Holding position until the trajectory starts");            // NOLINT
        break;

      case SampleError::SAMPLE_TIME_AFTER_END: {
        const double goal_tolerance = *rt_goal_tolerance_.readFromRT();
        const double goal_error = geodesic_error(trajectory->end_point().value(), system_state);
        RCLCPP_INFO(logger_, "Trajectory sample time is after trajectory end time.");  // NOLINT
        if (goal_tolerance > 0.0) {
          if (goal_error > goal_tolerance) {
            goal_tolerance_exceeded = true;
            RCLCPP_WARN(logger_, "Aborting trajectory. Terminal error exceeded threshold: %f", goal_error);  // NOLINT
          } else {
            trajectory_suceeded = true;
            RCLCPP_INFO(logger_, "Trajectory execution completed successfully");  // NOLINT
          }
        }
        rt_holding_position_.writeFromNonRT(true);
        RCLCPP_INFO(logger_, "Holding position until a new trajectory is received");  // NOLINT
      } break;

      default:
        // default to position hold
        rt_holding_position_.writeFromNonRT(true);
        break;
    }
  }

  if (active_goal) {
    // write feedback to the action server
    auto feedback = std::make_shared<FollowTrajectory::Feedback>();
    feedback->header.stamp = time;
    feedback->desired = reference_state;
    feedback->actual = system_state;
    feedback->error = error;
    active_goal->setFeedback(feedback);

    // check terminal conditions
    if (goal_tolerance_exceeded) {
      auto action_result = std::make_shared<FollowTrajectory::Result>();
      action_result->error_code = FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      action_result->error_string = "Trajectory execution aborted. Goal tolerance exceeded.";
      active_goal->setAborted(action_result);
      rt_holding_position_.writeFromNonRT(true);
    } else if (trajectory_suceeded) {
      auto action_result = std::make_shared<FollowTrajectory::Result>();
      action_result->error_code = FollowTrajectory::Result::SUCCESSFUL;
      action_result->error_string = "Trajectory execution completed successfully!";
      active_goal->setSucceeded(action_result);
      rt_holding_position_.writeFromNonRT(true);
    } else if (path_tolerance_exceeded) {
      auto action_result = std::make_shared<FollowTrajectory::Result>();
      action_result->error_code = FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      action_result->error_string = "Trajectory execution aborted. Path tolerance exceeded.";
      active_goal->setAborted(action_result);
      rt_holding_position_.writeFromNonRT(true);
    }
  }

  write_command(command_interfaces_, command_state);
  publish_controller_state();

  return controller_interface::return_type::OK;
}

}  // namespace geometric_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  geometric_trajectory_controller::GeometricTrajectoryController,
  controller_interface::ControllerInterface)
