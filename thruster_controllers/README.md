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

### Parameters

- thruster: The name of the thruster. [string]
- thrust_curve_coefficients: The thrust-to-PWM curve polynomial coefficients.
  These should be provided in the order of the lowest degree to the highest
  degree. [double array]
- min_thrust: The minimum thrust that can be produced by the thruster. [double]
- max_thrust: The maximum thrust that can be produced by the thruster. [double]
- min_deadband_pwm: The minimum PWM value in the deadband range for the
  thruster. [int]
- max_deadband_pwm: The maximum PWM value in the deadband range for the
  thruster. [int]
- neutral_pwm: A safe PWM value that is known to apply zero thrust. [int]
