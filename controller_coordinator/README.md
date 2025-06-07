# Controller Coordinator

The controller coordinator is a high-level interface for activating and
deactivating a control system. This is useful in scenarios where you want to
switch between a custom control framework and a company-provided control
framework.

## Clients

- controller_manager/set_hardware_component_state [controller_manager_msgs::srv::SetHardwareComponentState]
- controller_manager/switch_controller [controller_manager_msgs::srv::SwitchController]

## Services

- controller_coordinator/activate [std_srvs/srv/SetBool]
