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

#include "controller_common/common.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace end_effector_trajectory_controller
{

namespace
{

auto geodesic_error(const geometry_msgs::msg::Pose & goal, const geometry_msgs::msg::Pose & state) -> double
{
  Eigen::Isometry3d goal_mat, state_mat;
  tf2::fromMsg(goal, goal_mat);
  tf2::fromMsg(state, state_mat);

  return std::pow((goal.inverse() * state).log().norm(), 2);
}

}  // namespace

auto EndEffectorTrajectoryController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<end_effector_trajectory_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();
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
  dof_names_ = params_.joints;
  n_dofs_ = dof_names_.size();
  return controller_interface::CallbackReturn::SUCCESS;
}

auto EndEffectorTrajectoryController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  configure_parameters();

  end_effector_state_.writeFromNonRT(geometry_msgs::msg::Pose());
  first_sample_.writeFromNonRT(true);
  holding_position_.writeFromNonRT(false);
  trajectory_.writeFromNonRT(Trajectory());

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
      update_end_effector_state();
      trajectory_.writeFromNonRT(Trajectory(msg, end_effector_state_.readFromNonRT()));
      first_sample_.writeFromNonRT(true);
      holding_position_.writeFromNonRT(false);
    });

  // TODO(evan-palmer): add controller state
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

  std::ranges::transform(dof_names_, std::back_inserter(config.names), [](const auto & dof) {
    return params_.reference_controller.empty()
             ? std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION)
             : std::format("{}/{}/{}", params_.reference_controller, dof, hardware_interface::HW_IF_POSITION)
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

  std::ranges::transform(dof_names_, std::back_inserter(config.names), [](const auto & dof) {
    return std::format("{}/{}", dof, hardware_interface::HW_IF_POSITION);
  });

  return config;
}

auto EndEffectorTrajectoryController::update_end_effector_state() -> void
{
  if (params_.lookup_end_effector_transform) {
    try {
      const auto to_frame = params_.end_effector_frame_id;
      const auto from_frame = params_.odom_frame_id;
      const auto t = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);

      const auto * pose = end_effector_state_.readFromNonRT();
      *pose->position = t.transform.translation;
      *pose->orientation = t.transform.rotation;
    }
  } else if (!params_.use_external_measured_states) {
    auto get_value = [](const auto & interface) -> double {
      return interface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    };

    const auto * pose = end_effector_state_.readFromNonRT();
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

auto EndEffectorTrajectoryController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> controller_interface::return_type
{
  if (update_end_effector_state() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG(logger_, "Skipping controller update. Failed to update and validate interfaces");  // NOLINT
    return controller_interface::return_type::OK;
  }

  configure_parameters();

  // write a pose to the command interfaces
  auto write_command = [this](const geometry_msgs::msg::Pose & command) {
    const std::vector<double> vec = common::messages::to_vector(command);
    for (const auto & [i, dof] : std::views::enumerate(dof_names_)) {
      if (!command_interfaces_[i].set_value(vec[i])) {
        RCLCPP_WARN(logger_, "Failed to set command for joint %s", dof.c_str());  // NOLINT
      }
    }
  };

  // helper function used to hold the current end effector pose
  auto hold_position = [this]() {
    holding_position_.writeFromNonRT(true);
    write_command(*end_effector_state_.readFromRT());
  };

  // wait until a new trajectory is received
  if (*holding_position_.readFromRT()) {
    hold_position();
    return controller_interface::return_type::OK;
  }

  // set the sample time
  rclcpp::Time sample_time = time;
  if (*first_sample_.readFromRT()) {
    // this isn't the best way to do this, but there aren't any other options with this tooling:
    // https://github.com/ros-controls/ros2_controllers/issues/168
    // https://github.com/ros-controls/realtime_tools/issues/279
    first_sample_.writeFromNonRT(false);
    sample_time += period;
  }

  // we use the current sample to measure errors and the future sample as the setpoint
  // the future sample should be used to prevent the controller from lagging behind
  const auto * t = trajectory_.readFromRT();
  const geometry_msgs::msg::Pose sampled_state = t->sample(sample_time).value_or(geometry_msgs::msg::Pose());
  const auto sampled_command = t->sample(sample_time + update_period_);

  // if we experience an error when sampling the trajectory, handle the error and enter position hold
  if (!sampled_command.has_value()) {
    switch (sampled_command.error()) {
      case SampleError::SAMPLE_TIME_BEFORE_START:
        RCLCPP_WARN(logger_, "Sample time is before trajectory start time. Waiting for trajectory start");  // NOLINT

        // hold position but don't require a new trajectory
        hold_position();
        holding_position_.writeFromNonRT(false);
        break;

      case SampleError::SAMPLE_TIME_AFTER_END:
        RCLCPP_DEBUG(logger_, "Sample time is after trajectory end time");  // NOLINT
        if (params_.error_tolerance > 0.0) {
          const double terminal_error = geodesic_error(t->end_point().value(), *end_effector_state_.readFromRT());
          if (terminal_error <= params_.error_tolerance) {
            RCLCPP_INFO(logger_, "Successfully executed the trajectory");  // NOLINT
          } else {
            // NOLINTNEXTLINE
            RCLCPP_WARN(logger_, "Trajectory execution failed - reached trajectory end with error %f", terminal_error);
          }
        }
        // NOLINTNEXTLINE
        RCLCPP_INFO(logger_, "Trajectory execution complete. Holding position until a new trajectory is received");
        hold_position();
        break;

      case SampleError::EMPTY_TRAJECTORY:
        RCLCPP_WARN(logger_, "Trajectory is empty");                           // NOLINT
        RCLCPP_INFO(logger_, "Holding position until trajectory is received")  // NOLINT
        hold_position();
        break;
    }
    return controller_interface::return_type::OK;
  }

  const double error = geodesic_error(sampled_state, *end_effector_state_.readFromRT());
  if (error > params_.error_tolerance) {
    RCLCPP_WARN(logger_, "Aborting trajectory. Error threshold exceeded during execution: %f", error)  // NOLINT
    RCLCPP_INFO(logger_, "Holding position until a new trajectory is received")                        // NOLINT
    hold_position();
    return controller_interface::return_type::OK;
  }

  // we successfully sampled the trajectory
  const auto & command = result.value();
  write_command(result.value());

  return controller_interface::return_type::OK;
}

}  // namespace end_effector_trajectory_controller
