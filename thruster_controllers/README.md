# Thruster Controllers 
Thruster controllers convert the thrust calculated in previous controllers to something more useable for the hardware. This package contains controllers that perform this conversion. 

## Polynomial Thrust Curve Controller
Chainable controller to calculate the pwm command required to perform the thrust reference. 

### plugin library
thruster_controllers/polynomial_thrust_curve_controller

### References
Input to this controller is thrust [double].

### Commands
Output of this controller is the pwm required by the thruster to perform the action [double].

### Subscribers
polynomial_thrust_curve_controller/reference [std_msgs::msg::Float64]

### Publishers
polynomial_thrust_curve_controller/status [control_msgs::msg::SingleDOFStateStamped]

### Parameters
  - thruster: The name of the thruster. [string]
  - thrust_curve_coefficients: The thrust-to-PWM curve polynomial coefficients. These should be provided in the order of the lowest degree to the highest degree. [double array]
  - min_thrust: The minimum thrust that can be produced by the thruster. [double]
  - max_thrust: The maximum thrust that can be produced by the thruster. [double]
  - min_deadband_pwm: The minimum PWM value in the deadband range for the thruster. [int]
  - max_deadband_pwm: The maximum PWM value in the deadband range for the thruster. [int]
  - neutral_pwm: A safe PWM value that is known to apply zero thrust. [int]
