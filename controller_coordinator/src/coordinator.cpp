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

#include "coordinator.hpp"

#include <ranges>

#include "lifecycle_msgs/msg/state.hpp"

namespace coordinator
{

ControllerCoordinator::ControllerCoordinator()
: rclcpp::Node("controller_coordinator"),
  activate_hardware_request_(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>()),
  deactivate_hardware_request_(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>()),
  activate_controllers_request_(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>()),
  deactivate_controllers_request_(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>())
{
  param_listener_ = std::make_shared<controller_coordinator::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);

  // helper function used to wait for services to come up
  // this will block indefinitely
  auto wait_for_service = [this](const auto & client, const std::string & service_name) {
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for %s service to come up", service_name.c_str());  // NOLINT
    }
    RCLCPP_INFO(this->get_logger(), "%s service available", service_name.c_str());  // NOLINT
  };

  // create clients
  const std::string hardware_service = "controller_manager/set_hardware_component_state";
  hardware_client_ = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
    hardware_service, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(hardware_client_, hardware_service);

  const std::string switch_controller_name = "controller_manager/switch_controller";
  switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
    switch_controller_name, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(switch_controller_client_, switch_controller_name);

  // pre-configure the hardware activation/deactivation requests
  activate_hardware_request_->name = params_.hardware_interface;
  activate_hardware_request_->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

  deactivate_hardware_request_->name = params_.hardware_interface;
  deactivate_hardware_request_->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

  // pre-configure the controller activation/deactivation requests
  activate_controllers_request_->activate_controllers = params_.controller_sequence;
  activate_controllers_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  activate_controllers_request_->activate_asap = true;
  activate_controllers_request_->timeout = rclcpp::Duration::from_seconds(params_.timeout);

  deactivate_controllers_request_->deactivate_controllers = params_.controller_sequence;
  deactivate_controllers_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  deactivate_controllers_request_->activate_asap = true;
  deactivate_controllers_request_->timeout = rclcpp::Duration::from_seconds(params_.timeout);

  // create a service endpoint for users to activate or deactivate their system
  service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
  activate_system_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/activate",
    [this](
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      response->success = true;
      if (request->data) {
        RCLCPP_INFO(this->get_logger(), "Activating thruster hardware interface and controllers");  // NOLINT

        // activate the hardware interface
        hardware_client_->async_send_request(
          activate_hardware_request_,
          [this, &response](
            rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully activated thruster hardware interface");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to activate thruster hardware interface");  // NOLINT
              response->success = false;
              response->message = "Failed to activate thruster hardware interface";
            }
          });

        // activate the controllers
        switch_controller_client_->async_send_request(
          activate_controllers_request_,
          [this,
           &response](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully activated controllers");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to activate controllers");  // NOLINT
              response->success = false;
              response->message = "Failed to activate controllers";
            }
          });
      } else {
        RCLCPP_INFO(this->get_logger(), "Deactivating controllers and thruster hardware interface");  // NOLINT

        // deactivate the hardware interface
        hardware_client_->async_send_request(
          deactivate_hardware_request_,
          [this, &response](
            rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully deactivated thruster hardware interface");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to deactivate thruster hardware interface");  // NOLINT
              response->success = false;
              response->message = "Failed to deactivate thruster hardware interface";
            }
          });

        // deactivate the controllers
        switch_controller_client_->async_send_request(
          deactivate_controllers_request_,
          [this,
           &response](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully deactivated controllers");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to deactivate controllers");  // NOLINT
              response->success = false;
              response->message = "Failed to deactivate controllers";
            }
          });
      }
    },
    rclcpp::ServicesQoS(),
    service_callback_group_);
}

}  // namespace coordinator

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<coordinator::ControllerCoordinator>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
