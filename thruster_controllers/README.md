# Thruster Controllers

This package provides a collection of chainable thruster controllers, which
compute a hardware-level command (e.g., a PWM) that tracks a thruster reference
signal.

## Polynomial Thrust Curve Controller

A Chainable controller that calculates the PWM command required to track a
thrust reference using an N-degree polynomial thrust curve.

### Plugin Library

thruster_controllers/polynomial_thrust_curve_controller

### References

The input to this controller is thrust [double].

### Commands

The output of this controller is the PWM value required by the thruster to
perform the action [double].

### Subscribers

polynomial_thrust_curve_controller/reference [std_msgs::msg::Float64]

### Publishers

polynomial_thrust_curve_controller/status [control_msgs::msg::SingleDOFStateStamped]

## Gazebo Passthrough Controller

A Chainable controller that publishes a reference thrust value to a topic. To
use with Gazebo, set the published topic to be the `<thruster_model>/cmd_thrust`
topic subscribed to by Gazebo and launch a ROS-Gazebo bridge to proxy messages
between the two interfaces.

For example, given the thruster plugin configuration

```xml
<!-- example plugin configuration for a thruster -->
<plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
  <namespace>my_auv_model</namespace>
  <joint_name>thruster_joint</joint_name>
  <thrust_coefficient>-0.02</thrust_coefficient>
  <fluid_density>1000.0</fluid_density>
  <propeller_diameter>0.1</propeller_diameter>
  <velocity_control>true</velocity_control>
  <use_angvel_cmd>false</use_angvel_cmd>
</plugin>
```

the controller topic would be set to

```bash
/model/my_auv_model/joint/thruster_joint/cmd_thrust
```

with the ROS-Gazebo bridge:

```bash
/model/my_auv_model/joint/thruster_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double
```

### Plugin Library

thruster_controllers/gz_passthrough_controller

### References

The input to this controller is thrust [double].

### Commands

There are no command interfaces for this controller.

### Subscribers

gz_passthrough_controller/reference [std_msgs::msg::Float64]

### Publishers

gz_passthrough_controller/status [control_msgs::msg::SingleDOFStateStamped]
