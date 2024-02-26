// Copyright 2024, Evan Palmer
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

#include "thruster_allocation_matrix_controller/thruster_allocation_matrix_controller.hpp"

#include <Eigen/Dense>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace thruster_allocation_matrix_controller
{

namespace
{

Eigen::Vector6d create_six_dof_eigen_from_named_vector(
  const std::vector<std::string> & dof_names, const std::array<std::string, 6> & six_dof_names,
  const std::vector<double> & values)
{
  if (dof_names.size() != values.size()) {
    throw std::invalid_argument("The DoF names and values must have the same size.");
  }

  Eigen::Vector6d vec = Eigen::Vector6d::Zero();

  for (size_t i = 0; i < six_dof_names.size(); ++i) {
    auto it = std::find(dof_names.begin(), dof_names.end(), six_dof_names[i]);

    if (it == dof_names.end()) {
      vec[i] = std::numeric_limits<double>::quiet_NaN();
    } else {
      vec[i] = values[std::distance(dof_names.begin(), it)];
    }
  }

  return vec;
}

}  // namespace

ThrusterAllocationMatrixController::CallbackReturn ThrusterAllocationMatrixController::on_init()
{
  // Pulls parameters and updates them
  try {
    param_listener_ = std::make_shared<thruster_allocation_matrix_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    fprintf(stderr, "An exception occurred while initializing the controller: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ThrusterAllocationMatrixController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (std::int64_t i = 0; i < num_thrusters_; i++) {
    std::ostringstream temp_name;
    temp_name << "thruster_" << i;
    command_interfaces_config.names.emplace_back(temp_name.str() + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return command_interfaces_config;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Initialize the reference and state realtime messages
  const std::shared_ptr<geometry_msgs::msg::Wrench> reference_msg = std::make_shared<geometry_msgs::msg::Wrench>();
  reference_.writeFromNonRT(reference_msg);

  // Subscribe to the reference topic
  reference_sub_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Wrench> msg) { reference_callback(msg); });  // NOLINT

  // Setup the controller state publisher
  controller_state_pub_ =
    get_node()->create_publisher<control_msgs::msg::MultiDOFStateStamped>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>>(controller_state_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ThrusterAllocationMatrixController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // getting the reference forces and calculating the thrust forces using the tam
  const std::vector<double> reference_forces_values(reference_interfaces_.begin() + dof_, reference_interfaces_.end());
  auto reference_forces = create_six_dof_eigen_from_named_vector(dof_names_, six_dof_names_, reference_forces_values);

  Eigen::VectorXd thruster_forces = tam_.completeOrthogonalDecomposition().pseudoInverse() * reference_forces;

  // setting command interfaces with calculated thruster forces
  for (std::int64_t i = 0; i < num_thrusters_; i++) {
    command_interfaces_[i].set_value(thruster_forces[i]);
  }

  // TODO: we still aren't sure if this is the correct type/way of doing this. we may need to come back and do
  // something that makes more sense. (edit: this doesn't work)
  // if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
  //   rt_controller_state_pub_->msg_.header.stamp = time;

  //   rt_controller_state_pub_->msg_.dof_states[0].reference = reference_interfaces_;
  //   rt_controller_state_pub_->msg_.dof_states[0].time_step = period.seconds();
  //   rt_controller_state_pub_->msg_.dof_states[0].output = thruster_forces;

  //   rt_controller_state_pub_->unlockAndPublish();
  // }

  return controller_interface::return_type::OK;
}

bool ThrusterAllocationMatrixController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

std::vector<hardware_interface::CommandInterface> ThrusterAllocationMatrixController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(dof_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  for (size_t i = 0; i < dof_; ++i) {
    reference_interfaces.emplace_back(
      get_node()->get_name(), dof_names_[i] + "/" + hardware_interface::HW_IF_EFFORT, &reference_interfaces_[i]);
  }

  return reference_interfaces;
}

controller_interface::return_type ThrusterAllocationMatrixController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto * current_reference = reference_.readFromNonRT();

  // define wrench list and iterate through to assign it to interfaces
  std::vector<double> wrench = {(*current_reference)->force.x,  (*current_reference)->force.y,
                                (*current_reference)->force.z,  (*current_reference)->torque.x,
                                (*current_reference)->torque.y, (*current_reference)->torque.z};

  // Update the thrust reference
  for (size_t i = 0; i < wrench.size(); i++) {
    if (!std::isnan(wrench[i])) {
      reference_interfaces_[i] = wrench[i];
      wrench[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  return controller_interface::return_type::OK;
}

void ThrusterAllocationMatrixController::update_parameters()
{
  // Need this
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn ThrusterAllocationMatrixController::configure_parameters()
{
  update_parameters();

  // These are just used to improve readability
  dof_names_ = params_.dof_names;
  dof_ = dof_names_.size();

  // stroing each row of the tam matrix
  Eigen::Map<Eigen::VectorXd> tam_x(params_.tam.x.data(), params_.tam.x.size());
  Eigen::Map<Eigen::VectorXd> tam_y(params_.tam.y.data(), params_.tam.y.size());
  Eigen::Map<Eigen::VectorXd> tam_z(params_.tam.z.data(), params_.tam.z.size());
  Eigen::Map<Eigen::VectorXd> tam_rx(params_.tam.rx.data(), params_.tam.rx.size());
  Eigen::Map<Eigen::VectorXd> tam_ry(params_.tam.ry.data(), params_.tam.ry.size());
  Eigen::Map<Eigen::VectorXd> tam_rz(params_.tam.rz.data(), params_.tam.rz.size());

  // checking that each of the vectors is the same size
  num_thrusters_ = tam_x.size();
  if (
    (tam_y.size() != num_thrusters_) || (tam_z.size() != num_thrusters_) || (tam_rx.size() != num_thrusters_) ||
    (tam_ry.size() != num_thrusters_) || (tam_rz.size() != num_thrusters_)) {
    // if any of the tam vectors aren't the same size, then we need to notify the user and return failure
    RCLCPP_ERROR(get_node()->get_logger(), "Received vectors for TAM with differing lengths.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // otherwise, they are all the same length and we can pack them into an Eigen matrix
  Eigen::MatrixXd tam_(dof_, num_thrusters_);
  tam_ << tam_x, tam_y, tam_z, tam_rx, tam_ry, tam_rz;

  return controller_interface::CallbackReturn::SUCCESS;
}

void ThrusterAllocationMatrixController::reference_callback(std::shared_ptr<geometry_msgs::msg::Wrench> msg)
{
  try {
    reference_.writeFromNonRT(msg);
  }
  catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received an invalid reference message: %s", e.what());  // NOLINT
  }
}

}  // namespace thruster_allocation_matrix_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  thruster_allocation_matrix_controller::ThrusterAllocationMatrixController,
  controller_interface::ChainableControllerInterface)
