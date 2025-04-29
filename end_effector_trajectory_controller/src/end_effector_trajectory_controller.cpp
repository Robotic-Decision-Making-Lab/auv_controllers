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

namespace end_effector_trajectory_controller
{

controller_interface::return_type JointTrajectoryController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  auto logger = this->get_node()->get_logger();
  // update dynamic parameters
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(logger, params_);
    // update the PID gains
    // variable use_closed_loop_pid_adapter_ is updated in on_configure only
    if (use_closed_loop_pid_adapter_) {
      update_pids();
    }
  }

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new trajectory message has been received from Non-RT threads
  const auto current_trajectory_msg = current_trajectory_->get_trajectory_msg();
  auto new_external_msg = new_trajectory_msg_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (current_trajectory_msg != *new_external_msg && (*(rt_has_pending_goal_.readFromRT()) && !active_goal) == false) {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    current_trajectory_->update(*new_external_msg);
  }

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_state_interfaces(state_current_);

  // currently carrying out a trajectory
  if (has_active_trajectory()) {
    bool first_sample = false;
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    // if sampling the first time, set the point before you sample
    if (!current_trajectory_->is_sampled_already()) {
      first_sample = true;
      if (params_.interpolate_from_desired_state || params_.open_loop_control) {
        if (std::abs(last_commanded_time_.seconds()) < std::numeric_limits<float>::epsilon()) {
          last_commanded_time_ = time;
        }
        current_trajectory_->set_point_before_trajectory_msg(
          last_commanded_time_, last_commanded_state_, joints_angle_wraparound_);
      } else {
        current_trajectory_->set_point_before_trajectory_msg(time, state_current_, joints_angle_wraparound_);
      }
      traj_time_ = time;
    } else {
      traj_time_ += period;
    }

    // Sample expected state from the trajectory
    current_trajectory_->sample(traj_time_, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

    // Sample setpoint for next control cycle
    const bool valid_point = current_trajectory_->sample(
      traj_time_ + update_period_, interpolation_method_, command_next_, start_segment_itr, end_segment_itr, false);

    if (valid_point) {
      const rclcpp::Time traj_start = current_trajectory_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      double time_difference = traj_time_.seconds() - segment_time_from_start.seconds();
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      const bool before_last_point = end_segment_itr != current_trajectory_->end();
      auto active_tol = active_tolerances_.readFromRT();

      // have we reached the end, are not holding position, and is a timeout configured?
      // Check independently of other tolerances
      if (
        !before_last_point && *(rt_is_holding_.readFromRT()) == false && cmd_timeout_ > 0.0 &&
        time_difference > cmd_timeout_) {
        RCLCPP_WARN(logger, "Aborted due to command timeout");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index) {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        // print output per default, goal will be aborted afterwards
        if (
          (before_last_point || first_sample) && *(rt_is_holding_.readFromRT()) == false &&
          !check_state_tolerance_per_joint(
            state_error_, index, active_tol->state_tolerance[index], true /* show_errors */)) {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point && *(rt_is_holding_.readFromRT()) == false &&
          !check_state_tolerance_per_joint(
            state_error_, index, active_tol->goal_state_tolerance[index], false /* show_errors */)) {
          outside_goal_tolerance = true;

          if (active_tol->goal_time_tolerance != 0.0) {
            // if we exceed goal_time_tolerance set it to aborted
            if (time_difference > active_tol->goal_time_tolerance) {
              within_goal_time = false;
              // print once, goal will be aborted afterwards
              check_state_tolerance_per_joint(
                state_error_, index, default_tolerances_.goal_state_tolerance[index], true /* show_errors */);
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time) {
        if (use_closed_loop_pid_adapter_) {
          // Update PIDs
          for (auto i = 0ul; i < num_cmd_joints_; ++i) {
            // If effort interface only, add desired effort as feed forward
            // If velocity interface, ignore desired effort
            size_t index_cmd_joint = map_cmd_to_joints_[i];
            tmp_command_[index_cmd_joint] =
              (command_next_.velocities[index_cmd_joint] * ff_velocity_scale_[i]) +
              (has_effort_command_interface_ ? command_next_.effort[index_cmd_joint] : 0.0) +
              pids_[i]->compute_command(
                state_error_.positions[index_cmd_joint], state_error_.velocities[index_cmd_joint], period);
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_) {
          assign_interface_from_point(joint_command_interface_[0], command_next_.positions);
        }
        if (has_velocity_command_interface_) {
          if (use_closed_loop_pid_adapter_) {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          } else {
            assign_interface_from_point(joint_command_interface_[1], command_next_.velocities);
          }
        }
        if (has_acceleration_command_interface_) {
          assign_interface_from_point(joint_command_interface_[2], command_next_.accelerations);
        }
        if (has_effort_command_interface_) {
          if (use_closed_loop_pid_adapter_) {
            assign_interface_from_point(joint_command_interface_[3], tmp_command_);
          } else {
            // If position and effort command interfaces, only pass desired effort
            assign_interface_from_point(joint_command_interface_[3], state_desired_.effort);
          }
        }

        // store the previous command and time used in open-loop control mode
        last_commanded_state_ = command_next_;
        last_commanded_time_ = time;
      }

      if (active_goal) {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving) {
          auto result = std::make_shared<FollowJTrajAction::Result>();
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          result->set__error_string("Aborted due to path tolerance violation");
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          rt_has_pending_goal_.writeFromNonRT(false);

          RCLCPP_WARN(logger, "Aborted due to state tolerance violation");

          new_trajectory_msg_.reset();
          new_trajectory_msg_.initRT(set_hold_position());
        }
        // check goal tolerance
        else if (!before_last_point) {
          if (!outside_goal_tolerance) {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            result->set__error_string("Goal successfully reached!");
            active_goal->setSucceeded(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_.writeFromNonRT(false);

            RCLCPP_INFO(logger, "Goal reached, success!");

            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_success_trajectory_point());
          } else if (!within_goal_time) {
            const std::string error_string =
              "Aborted due to goal_time_tolerance exceeding by " + std::to_string(time_difference) + " seconds";

            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            result->set__error_string(error_string);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_.writeFromNonRT(false);

            RCLCPP_WARN(logger, "%s", error_string.c_str());

            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_hold_position());
          }
        }
      } else if (tolerance_violated_while_moving && *(rt_has_pending_goal_.readFromRT()) == false) {
        // we need to ensure that there is no pending goal -> we get a race condition otherwise
        RCLCPP_ERROR(logger, "Holding position due to state tolerance violation");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      } else if (!before_last_point && !within_goal_time && *(rt_has_pending_goal_.readFromRT()) == false) {
        RCLCPP_ERROR(logger, "Exceeded goal_time_tolerance: holding position...");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }
      // else, run another cycle while waiting for outside_goal_tolerance
      // to be satisfied (will stay in this state until new message arrives)
      // or outside_goal_tolerance violated within the goal_time_tolerance
    }
  }

  publish_state(time, state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

}  // namespace end_effector_trajectory_controller
