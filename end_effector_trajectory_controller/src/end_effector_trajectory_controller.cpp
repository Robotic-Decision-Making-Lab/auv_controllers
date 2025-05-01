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

#include "end_effector_trajectory_controller/end_effector_trajectory_controller.hpp"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "controller_common/common.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace end_effector_trajectory_controller
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

auto EndEffectorTrajectoryController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<end_effector_trajectory_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();
  return controller_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto EndEffectorTrajectoryController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto EndEffectorTrajectoryController::configure_parameters() -> controller_interface::CallbackReturn
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

auto EndEffectorTrajectoryController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  end_effector_state_.writeFromNonRT(geometry_msgs::msg::Pose());
  command_interfaces_.reserve(n_dofs_);
  update_period_ = rclcpp::Duration(0.0, static_cast<uint32_t>(1.0e9 / static_cast<double>(get_update_rate())));

  if (params_.use_external_measured_states) {
    end_effector_state_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
      "~/end_effector_state",
      rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Pose> msg) {  // NOLINT
        end_effector_state_.writeFromNonRT(*msg);
      });
  } else {
    if (params_.lookup_end_effector_transform) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }
  }

  trajectory_sub_ = get_node()->create_subscription<auv_control_msgs::msg::EndEffectorTrajectory>(
    "~/trajectory",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<auv_control_msgs::msg::EndEffectorTrajectory> msg) {  // NOLINT
      auto updated_msg = *msg;
      if (!validate_trajectory(updated_msg)) {
        RCLCPP_ERROR(logger_, "Ignoring invalid trajectory message");  // NOLINT
        return;
      }
      rt_trajectory_.writeFromNonRT(Trajectory(updated_msg, *end_effector_state_.readFromNonRT()));
      rt_first_sample_.writeFromNonRT(true);
      rt_holding_position_.writeFromNonRT(false);
    });

  auto handle_goal =
    [this](const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const FollowTrajectoryAction::Goal> goal) {
      RCLCPP_INFO(logger_, "Received new trajectory goal");  // NOLINT
      if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_ERROR(logger_, "Can't accept new action goals. Controller is not running.");
        return rclcpp_action::GoalResponse::REJECT;
      }
      auto updated_msg = goal->trajectory;  // make a non-const copy
      if (!validate_trajectory(updated_msg)) {
        RCLCPP_ERROR(logger_, "Ignoring invalid trajectory message");  // NOLINT
        return rclcpp_action::GoalResponse::REJECT;
      }
      rt_trajectory_.writeFromNonRT(Trajectory(updated_msg, *end_effector_state_.readFromNonRT()));
      rt_first_sample_.writeFromNonRT(true);
      rt_holding_position_.writeFromNonRT(false);
      RCLCPP_INFO(logger_, "Accepted new trajectory goal");  // NOLINT
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  auto handle_cancel = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectoryAction>> gh) {
    RCLCPP_INFO(logger_, "Received cancel action goal");  // NOLINT
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal && active_goal->gh_ == gh) {
      RCLCPP_INFO(logger_, "Canceling active goal");  // NOLINT
      auto action_result = std::make_shared<FollowTrajectoryAction::Result>();
      active_goal->setCanceled(action_result);
      rt_holding_position_.writeFromNonRT(true);
      rt_first_sample_.writeFromNonRT(true);
      rt_goal_in_progress_.writeFromNonRT(false);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectoryAction>> gh) {
    RCLCPP_INFO(logger_, "Received accepted action goal");  // NOLINT
    rt_goal_in_progress_.writeFromNonRT(true);
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal) {
      RCLCPP_INFO(logger_, "Canceling active goal");  // NOLINT
      auto action_result = std::make_shared<FollowTrajectoryAction::Result>();
      action_result->error_code = FollowTrajectoryAction::Result::INVALID_GOAL;
      action_result->error_string = "Current goal cancelled by a new incoming action.";
      active_goal->setCanceled(action_result);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }

    // TODO(evan-palmer): update active tolerances

    RealtimeGoalHandlePtr rt_gh = std::make_shared<RealtimeGoalHandle>(gh);
    rt_gh->execute();
    rt_active_goal_.writeFromNonRT(rt_gh);

    goal_handle_timer_.reset();
    goal_handle_timer_ = get_node()->create_wall_timer(action_monitor_period_, [rt_gh]() { rt_gh->runNonRealtime(); });
  };

  action_server_ = rclcpp_action::create_server<auv_control_msgs::action::FollowEndEffectorTrajectory>(
    get_node(), "~/follow_trajectory", handle_goal, handle_cancel, handle_accepted);

  controller_state_pub_ = get_node()->create_publisher<ControllerState>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerState>>(controller_state_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto EndEffectorTrajectoryController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  rt_first_sample_.writeFromNonRT(true);
  rt_holding_position_.writeFromNonRT(true);  // hold position until a trajectory is received
  common::messages::reset_message(end_effector_state_.readFromNonRT());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto EndEffectorTrajectoryController::command_interface_configuration() const
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

auto EndEffectorTrajectoryController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  if (params_.use_external_measured_states || params_.lookup_end_effector_transform) {
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

auto EndEffectorTrajectoryController::update_end_effector_state() -> controller_interface::return_type
{
  if (params_.lookup_end_effector_transform) {
    const auto to_frame = params_.end_effector_frame_id;
    const auto from_frame = params_.odom_frame_id;
    try {
      const auto t = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);

      auto * pose = end_effector_state_.readFromNonRT();
      pose->position.x = t.transform.translation.x;
      pose->position.y = t.transform.translation.y;
      pose->position.z = t.transform.translation.z;
      pose->orientation = t.transform.rotation;
    }
    catch (const tf2::TransformException & ex) {
      const auto err = std::format("Failed to get transform from {} to {}: {}", to_frame, from_frame, ex.what());
      RCLCPP_DEBUG(logger_, err.c_str());  // NOLINT
      return controller_interface::return_type::ERROR;
    }
  } else if (!params_.use_external_measured_states) {
    auto get_value = [](const auto & interface) -> double {
      return interface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    };
    auto * pose = end_effector_state_.readFromNonRT();
    pose->position.x = get_value(state_interfaces_[0]);
    pose->position.y = get_value(state_interfaces_[1]);
    pose->position.z = get_value(state_interfaces_[2]);
    pose->orientation.x = get_value(state_interfaces_[3]);
    pose->orientation.y = get_value(state_interfaces_[4]);
    pose->orientation.z = get_value(state_interfaces_[5]);
    pose->orientation.w = get_value(state_interfaces_[6]);
  }

  if (common::math::has_nan(common::messages::to_vector(*end_effector_state_.readFromNonRT()))) {
    RCLCPP_DEBUG(logger_, "Received end effector state with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

auto EndEffectorTrajectoryController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  if (update_end_effector_state() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. Failed to update and validate interfaces");  // NOLINT
    return controller_interface::return_type::OK;
  }

  const auto active_goal = *rt_active_goal_.readFromRT();
  const bool goal_in_progress = *rt_goal_in_progress_.readFromRT();
  if (goal_in_progress && active_goal == nullptr) {
    RCLCPP_DEBUG(logger_, "Goal in progress but no active goal. Ignoring update");  // NOLINT
    return controller_interface::return_type::OK;
  }

  // hold position until a new trajectory is received
  if (*rt_holding_position_.readFromRT()) {
    hold_position(true);
    return controller_interface::return_type::OK;
  }

  // set the sample time
  rclcpp::Time sample_time = time;
  if (*rt_first_sample_.readFromRT()) {
    rt_trajectory_.readFromRT()->reset_initial_state(*end_effector_state_.readFromRT());
    rt_first_sample_.writeFromNonRT(false);
    sample_time += period;
  }

  const geometry_msgs::msg::Pose & state = *end_effector_state_.readFromRT();

  // we use the current sample to measure errors and the future sample as the command
  // the future sample should be used in order to prevent the controller from lagging
  const auto * trajectory = rt_trajectory_.readFromRT();
  const auto sampled_reference = trajectory->sample(sample_time);
  const auto sampled_command = trajectory->sample(sample_time + update_period_);

  if (sampled_command.has_value()) {
    const auto [command, segment] = sampled_command.value();
    const auto [start, end] = segment;

    // TODO(evan-palmer): is this going to be used?
    const rclcpp::Time segment_start_time = trajectory->start_time() + start.time_from_start;
    const double time_difference = sample_time.seconds() - segment_start_time.seconds();

    geometry_msgs::msg::Pose sampled_state;
    double error = std::numeric_limits<double>::quiet_NaN();

    // check the path tolerance
    if (sampled_reference.has_value()) {
      const auto [reference, reference_segment] = sampled_reference.value();
      const double path_tolerance = *rt_path_tolerance_.readFromRT();

      error = geodesic_error(reference, state);
      sampled_state = reference;

      if (path_tolerance > 0.0 && error > path_tolerance) {
        RCLCPP_WARN(logger_, "Aborting trajectory. Error threshold exceeded during execution: %f", error);  // NOLINT
        RCLCPP_INFO(logger_, "Holding position until a new trajectory is received");                        // NOLINT
        hold_position(true);
        return controller_interface::return_type::OK;
      }
    }

    write_command(command_interfaces_, command);
    if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
      rt_controller_state_pub_->msg_.header.stamp = get_node()->now();
      rt_controller_state_pub_->msg_.reference = sampled_state;
      rt_controller_state_pub_->msg_.feedback = state;
      rt_controller_state_pub_->msg_.error = error;
      rt_controller_state_pub_->msg_.output = command;
      rt_controller_state_pub_->unlockAndPublish();
    }
  } else {
    switch (sampled_command.error()) {
      case SampleError::SAMPLE_TIME_BEFORE_START:
        // hold position without required a new trajectory to be sent
        hold_position(false);
        break;

      case SampleError::SAMPLE_TIME_AFTER_END: {
        const double goal_tolerance = *rt_goal_tolerance_.readFromRT();
        const double goal_error = geodesic_error(trajectory->end_point().value(), state);

        if (goal_tolerance > 0.0 && goal_error > goal_tolerance) {
          // TODO(evan-palmer): fix
        }
      } break;

      default:
        break;
    }
  }

  // if (!sampled_command.has_value()) {
  //   // if we experience an error when sampling the trajectory, handle the error and enter position hold
  //   switch (sampled_command.error()) {
  //     case SampleError::SAMPLE_TIME_BEFORE_START:
  //       // NOLINTNEXTLINE
  //       RCLCPP_WARN_ONCE(logger_, "Sample time is before trajectory start time. Waiting for trajectory start");
  //       // TODO(evan-palmer): write feedback here?
  //       // hold position but don't require a new trajectory
  //       hold_position();
  //       rt_holding_position_.writeFromNonRT(false);
  //       break;

  //     case SampleError::SAMPLE_TIME_AFTER_END:
  //       RCLCPP_DEBUG(logger_, "Sample time is after trajectory end time");  // NOLINT
  //       {
  //         const double goal_tolerance = *rt_goal_tolerance_.readFromRT();
  //         if (goal_tolerance > 0.0) {
  //           const double terminal_error = geodesic_error(t->end_point().value(), state);
  //           if (terminal_error <= goal_tolerance) {
  //             if (active_goal) {
  //               auto result = std::make_shared<FollowTrajectoryAction::Result>();
  //               result->error_code = FollowTrajectoryAction::Result::SUCCESSFUL;
  //               result->error_string = "Trajectory execution completed successfully";
  //               active_goal->setSucceeded(result);
  //               rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  //               rt_goal_in_progress_.writeFromNonRT(false);
  //             }
  //             RCLCPP_INFO(logger_, "Successfully executed the trajectory");  // NOLINT
  //           } else {
  //             if (active_goal) {
  //               auto result = std::make_shared<FollowTrajectoryAction::Result>();
  //               result->error_code = FollowTrajectoryAction::Result::GOAL_TOLERANCE_VIOLATED;
  //               result->error_string = "Trajectory execution completed with end effector pose outside of tolerance";
  //               active_goal->setAborted(result);
  //               rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  //               rt_goal_in_progress_.writeFromNonRT(false);
  //             }
  //             // NOLINTNEXTLINE
  //             RCLCPP_WARN(logger_, "Trajectory execution failed. Reached trajectory end with error %f",
  //             terminal_error);
  //           }
  //         }
  //       }
  //       // NOLINTNEXTLINE
  //       RCLCPP_INFO(logger_, "Trajectory execution complete. Holding position until a new trajectory is received");
  //       hold_position();
  //       break;
  //   }
  //   return controller_interface::return_type::OK;
  // }

  return controller_interface::return_type::OK;
}

auto EndEffectorTrajectoryController::hold_position(bool continue_hold) -> void
{
  // this is called only after the end effector state has been updated and validated, so we can assume that the state
  // command will be valid
  rt_holding_position_.writeFromNonRT(continue_hold);
  const geometry_msgs::msg::Pose state = *end_effector_state_.readFromRT();
  write_command(command_interfaces_, state);
  geometry_msgs::msg::Pose reference;
  common::messages::reset_message(&reference);

  // publish the controller state here so that we don't have to do it every time this is called in the update function
  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = get_node()->now();
    rt_controller_state_pub_->msg_.reference = reference;
    rt_controller_state_pub_->msg_.feedback = state;
    rt_controller_state_pub_->msg_.error = std::numeric_limits<double>::quiet_NaN();
    rt_controller_state_pub_->msg_.output = state;
    rt_controller_state_pub_->unlockAndPublish();
  }
}

auto EndEffectorTrajectoryController::validate_trajectory(
  auv_control_msgs::msg::EndEffectorTrajectory & trajectory) const -> bool
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
  } else {
    trajectory.header.stamp = get_node()->now();
  }

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

}  // namespace end_effector_trajectory_controller
