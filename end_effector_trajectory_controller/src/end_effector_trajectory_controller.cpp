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
  return controller_interface::CallbackReturn::SUCCESS;
}

auto EndEffectorTrajectoryController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  end_effector_state_.writeFromNonRT(geometry_msgs::msg::Pose());
  first_sample_.writeFromNonRT(true);
  holding_position_.writeFromNonRT(false);

  command_interfaces_.reserve(n_dofs_);

  // use the update period to sample the "next" trajectory point
  update_period_ = rclcpp::Duration(0.0, static_cast<uint32_t>(1.0e9 / static_cast<double>(get_update_rate())));

  // the states can be captured in one of three ways:
  // 1. using the topic interface - when available, this is preferred over the tf2 interface
  // 2. using the state interfaces - this is the default, but often not available
  // 3. using tf2 - this is the most common interface, but requires a transform to be published and is not as robust
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

  // at the moment, the only way to set the current trajectory is via a topic
  // TODO(evan-palmer): implement an action server to execute trajectories
  trajectory_sub_ = get_node()->create_subscription<auv_control_msgs::msg::EndEffectorTrajectory>(
    "~/trajectory",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<auv_control_msgs::msg::EndEffectorTrajectory> msg) {  // NOLINT
      update_end_effector_state();
      trajectory_.writeFromNonRT(Trajectory(msg, *end_effector_state_.readFromNonRT()));
      first_sample_.writeFromNonRT(true);
      holding_position_.writeFromNonRT(false);
    });

  controller_state_pub_ = get_node()->create_publisher<auv_control_msgs::msg::EndEffectorTrajectoryControllerState>(
    "~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<auv_control_msgs::msg::EndEffectorTrajectoryControllerState>>(
      controller_state_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto EndEffectorTrajectoryController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
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
    try {
      const auto to_frame = params_.end_effector_frame_id;
      const auto from_frame = params_.odom_frame_id;
      const auto t = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);

      auto * pose = end_effector_state_.readFromNonRT();
      pose->position.x = t.transform.translation.x;
      pose->position.y = t.transform.translation.y;
      pose->position.z = t.transform.translation.z;
      pose->orientation = t.transform.rotation;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(  // NOLINT
        logger_,
        "Failed to get transform from %s to %s: %s",
        params_.end_effector_frame_id.c_str(),
        params_.odom_frame_id.c_str(),
        ex.what());  // NOLINT
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
    RCLCPP_DEBUG(logger_, "Received system state with NaN value.");  // NOLINT
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto EndEffectorTrajectoryController::publish_controller_state(
  const geometry_msgs::msg::Pose & reference,
  const geometry_msgs::msg::Pose & feedback,
  double error,
  const geometry_msgs::msg::Pose & output) -> void
{
  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = get_node()->now();
    rt_controller_state_pub_->msg_.reference = reference;
    rt_controller_state_pub_->msg_.feedback = feedback;
    rt_controller_state_pub_->msg_.error = error;
    rt_controller_state_pub_->msg_.output = output;
    rt_controller_state_pub_->unlockAndPublish();
  }
}

auto EndEffectorTrajectoryController::hold_position() -> void
{
  holding_position_.writeFromNonRT(true);
  const geometry_msgs::msg::Pose state = *end_effector_state_.readFromRT();
  write_command(command_interfaces_, state);
  geometry_msgs::msg::Pose reference;
  common::messages::reset_message(&reference);
  publish_controller_state(reference, state, std::numeric_limits<double>::quiet_NaN(), state);
}

auto EndEffectorTrajectoryController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  if (update_end_effector_state() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. Failed to update and validate interfaces");  // NOLINT
    return controller_interface::return_type::OK;
  }

  configure_parameters();

  const geometry_msgs::msg::Pose & state = *end_effector_state_.readFromRT();

  // continue holding position until a new trajectory is received
  if (*holding_position_.readFromRT()) {
    hold_position();
    return controller_interface::return_type::OK;
  }

  // wait until a trajectory is received
  // the above condition should prevent this from happening, but we add a check just to be safe
  if (trajectory_.readFromRT() == nullptr) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. No trajectory received");  // NOLINT
    return controller_interface::return_type::OK;
  }

  // set the sample time
  rclcpp::Time sample_time = time;
  if (*first_sample_.readFromRT()) {
    first_sample_.writeFromNonRT(false);
    sample_time += period;
  }

  // we use the current sample to measure errors and the future sample as the command
  // the future sample should be used in order to prevent the controller from lagging
  const auto * t = trajectory_.readFromRT();
  const geometry_msgs::msg::Pose sampled_state = t->sample(sample_time).value_or(geometry_msgs::msg::Pose());
  const auto sampled_command = t->sample(sample_time + update_period_);

  if (!sampled_command.has_value()) {
    // if we experience an error when sampling the trajectory, handle the error and enter position hold
    switch (sampled_command.error()) {
      case SampleError::SAMPLE_TIME_BEFORE_START:
        // NOLINTNEXTLINE
        RCLCPP_WARN_ONCE(logger_, "Sample time is before trajectory start time. Waiting for trajectory start");

        // hold position but don't require a new trajectory
        hold_position();
        holding_position_.writeFromNonRT(false);
        break;

      case SampleError::SAMPLE_TIME_AFTER_END:
        RCLCPP_DEBUG(logger_, "Sample time is after trajectory end time");  // NOLINT
        if (params_.error_tolerance > 0.0) {
          const double terminal_error = geodesic_error(t->end_point().value(), state);
          if (terminal_error <= params_.error_tolerance) {
            RCLCPP_INFO(logger_, "Successfully executed the trajectory");  // NOLINT
          } else {
            // NOLINTNEXTLINE
            RCLCPP_WARN(logger_, "Trajectory execution failed. Reached trajectory end with error %f", terminal_error);
          }
        }
        // NOLINTNEXTLINE
        RCLCPP_INFO(logger_, "Trajectory execution complete. Holding position until a new trajectory is received");
        hold_position();
        break;

      case SampleError::EMPTY_TRAJECTORY:
        RCLCPP_WARN(logger_, "Trajectory is empty");                            // NOLINT
        RCLCPP_INFO(logger_, "Holding position until trajectory is received");  // NOLINT
        hold_position();
        break;
    }
    return controller_interface::return_type::OK;
  }

  // check to see if the current state is too far from the sampled state
  const double error = geodesic_error(sampled_state, state);
  if (params_.error_tolerance > 0.0) {
    if (error > params_.error_tolerance) {
      RCLCPP_WARN(logger_, "Aborting trajectory. Error threshold exceeded during execution: %f", error);  // NOLINT
      RCLCPP_INFO(logger_, "Holding position until a new trajectory is received");                        // NOLINT
      hold_position();
      return controller_interface::return_type::OK;
    }
  }

  // we successfully sampled the trajectory
  const geometry_msgs::msg::Pose command = sampled_command.value();
  write_command(command_interfaces_, command);
  publish_controller_state(sampled_state, state, error, command);

  return controller_interface::return_type::OK;
}

}  // namespace end_effector_trajectory_controller
