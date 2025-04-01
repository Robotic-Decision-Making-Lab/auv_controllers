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

#include "velocity_controllers/adaptive_integral_terminal_sliding_mode_controller.hpp"

namespace velocity_controllers
{

namespace
{

void reset_twist_msg(geometry_msgs::msg::Twist * msg)
{
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}

auto twist_to_vector(const geometry_msgs::msg::Twist & twist) -> std::vector<double>
{
  return {
    twist.linear.x,
    twist.linear.y,
    twist.linear.z,
    twist.angular.x,
    twist.angular.y,
    twist.angular.z,
  };
}

}  // namespace

auto AdaptiveIntegralTerminalSlidingModeController::on_init() -> controller_interface::CallbackReturn override
{
  param_listener_ = std::make_shared<adaptive_integral_terminal_sliding_mode_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto AdaptiveIntegralTerminalSlidingModeController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();

  // Store the controller gains
  dofs_ = params_.joints;
  std::vector<double> k1, k2, k1_min, s_min, p;
  for (const auto & dof : dofs_) {
    k1.push_back(params_.gains.joints_map[dof].k1);
    k1_min.push_back(params_.gains.joints_map[dof].k1_min);
    k2.push_back(params_.gains.joints_map[dof].k2);
    s_min.push_back(params_.gains.joints_map[dof].s_min);
    p.push_back(params_.gains.joints_map[dof].p);
  }
  k1_ = Eigen::Vector6d(k1.data()).asDiagonal().toDenseMatrix();
  k2_ = Eigen::Vector6d(k2.data()).asDiagonal().toDenseMatrix();
  k1_min_ = Eigen::Vector6d(k1_min.data()).asDiagonal().toDenseMatrix();
  s_min_ = Eigen::Vector6d(s_min.data()).asDiagonal().toDenseMatrix();
  p_ = Eigen::Vector6d(p.data()).asDiagonal().toDenseMatrix();

  // Update the hydrodynamic parameters
  // TODO(evan-palmer): Read these from the robot_description instead of a parameters file
  const Eigen::Vector3d moments_of_inertia(params_.hydrodynamics.moments_of_inertia.data());
  const Eigen::Vector6d added_mass(params_.hydrodynamics.added_mass.data());
  Eigen::Vector6d linear_damping(params_.hydrodynamics.linear_damping.data());
  Eigen::Vector6d quadratic_damping(params_.hydrodynamics.quadratic_damping.data());
  Eigen::Vector3d cob(params_.hydrodynamics.center_of_buoyancy.data());
  Eigen::Vector3d cog(params_.hydrodynamics.center_of_gravity.data());

  inertia_ = std::make_unique<hydrodynamics::Inertia>(params_.hydrodynamics.mass, moments_of_inertia, added_mass);
  coriolis_ = std::make_unique<hydrodynamics::Coriolis>(params_.hydrodynamics.mass, moments_of_inertia, added_mass);
  damping_ = std::make_unique<hydrodynamics::Damping>(std::move(linear_damping), std::move(quadratic_damping));
  restoring_forces_ = std::make_unique<hydrodynamics::RestoringForces>(
    params_.hydrodynamics.weight, params_.hydrodynamics.buoyancy, std::move(cob), std::move(cog));

  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn override
{
  auto ret = configure_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  reference_.writeFromNonRT(geometry_msgs::msg::Twist());
  command_interaces_.resize(dofs_.size());
  system_state_values_.resize(dofs_.size(), std::numeric_limits<double>::quiet_NaN());

  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/reference",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {  // NOLINT
      reference_.writeFromNonRT(*msg);
    });  // NOLINT

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn override
{
  system_rotation_.writeFromNonRT(Eigen::Quaterniond::Identity());
  reset_twist_msg(reference_.readFromNonRT());
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto AdaptiveIntegralTerminalSlidingModeController::command_interface_configuration() const
  -> controller_interface::InterfaceConfiguration override
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(dofs_.size());

  for (const auto & dof : dofs_) {
    config.names.emplace_back(
      params_.reference_controller.empty()
        ? std::format("{}/{}", dof, hardware_interface::HW_IF_EFFORT)
        : std::format("{}/{}/{}", params_.reference_controller, dof, hardware_interface::HW_IF_EFFORT));
  }

  return config;
}

auto AdaptiveIntegralTerminalSlidingModeController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration override
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(dofs_.size());

  for (const auto & dof : dofs_) {
    config.names.emplace_back(std::format("{}/{}", dof, hardware_interface::HW_IF_VELOCITY));
  }

  return config;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_export_reference_interfaces()
  -> std::vector<hardware_interface::CommandInterface> override
{
  reference_interfaces_.resize(dofs_.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());

  for (const auto & dof : dofs_) {
    interfaces.emplace_back(
      get_node()->get_name(),
      std::format("{}/{}", dof, hardware_interface::HW_IF_EFFORT),
      &reference_interfaces_[interfaces.size()]);
  }

  return interfaces;
}

auto AdaptiveIntegralTerminalSlidingModeController::on_set_chained_mode(bool chained_mode) -> bool override
{
  return true;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();
  const std::vector<double> reference = twist_to_vector(*current_reference);
  for (std::size_t i = 0; i < reference.size(); ++i) {
    if (!std::isnan(reference[i])) {
      reference_interfaces_[i] = reference[i];
    }
  }
  reset_twist_msg(current_reference);
  return controller_interface::return_type::OK;
}

auto AdaptiveIntegralTerminalSlidingModeController::update_system_state_values() -> controller_interface::return_type
{
  for (std::size_t i = 0; i < system_state_values_.size(); ++i) {
    const auto value = state_interfaces_[i].get_optional();
    if (!value.has_value()) {
      RCLCPP_WARN(get_node()->get_logger(), std::format("Failed to get state value for joint {}", dofs_[i]).c_str());
      return controller_interface::return_type::ERROR;
    }
    system_state_values_[i] = value.value();
  }
}

auto AdaptiveIntegralTerminalSlidingModeController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) -> controller_interface::return_type override
{
  configure_parameters();

  update_system_state_values();
  const Eigen::Vector6d velocity_state(system_state_values_.data());

  // Calculate the velocity error
  std::vector<double> velocity_error_values;
  velocity_error_values.reserve(DOF);
  std::ranges::transform(
    reference_interfaces_,
    system_state_values_,
    std::back_inserter(velocity_error_values),
    [](double reference, double state) {
      return !std::isnan(reference) && !std::isnan(state) ? reference - state
                                                          : std::numeric_limits<double>::quiet_NaN();
    });

  if (std::ranges::all_of(velocity_error_values, [](double i) { return std::isnan(i); })) {
    RCLCPP_DEBUG(get_node()->get_logger(), "All velocity error values are NaN. Skipping control update.");
    return controller_interface::return_type::OK;
  }

  const Eigen::Vector6d velocity_error(velocity_error_values.data());

  // Try getting the latest orientation of the vehicle in the inertial frame. If the transform is not available, use the
  // last known orientation.
  try {
    const geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform(inertial_frame_id_, vehicle_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(t.transform.rotation, *system_rotation_.readFromRT());
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      std::format(
        "Could not transform {} to {} using latest available transform, {}",
        inertial_frame_id_,
        vehicle_frame_id_,
        e.what())
        .c_str());
  }

  // TODO(evan-palmer): Add sign boundary thickness parameter
  // TODO(evan-palmer): add root
  const Eigen::Vector6d e_i_dot = velocity_error.unaryExpr([this](double x) { return std::tanh(x / 0.01) });
  e_i_dot *= velocity_error.unaryExpr([this](double x) { return std::abs(x) });

  // calculate the computed torque control
  const Eigen::Vector6d t0 =
    inertia_->mass_matrix * (p_ * e_i_dot) + coriolis_->calculate_coriolis_matrix(velocity_state) * velocity_state +
    damping_->calculate_damping_matrix(velocity_state) * velocity_state +
    restoring_forces_->calculate_restoring_forces_vector(system_rotation_.readFromRT()->toRotationMatrix());

  // calculate the adaptive disturbance rejection control
  // TODO(evan-palmer): discretize dynamics
  const Eigen::Vector6d t1 = -k1_ *
}

}  // namespace velocity_controllers
