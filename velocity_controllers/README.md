# Velocity Controllers
A velocity controller is an automatic means of controlling the velocity of the vehicle. The controllers in this package should recieve a velocity reference and return a wrench command.

## Integral Sliding Mode Controller
  - Chainable controller to calculate the acceleration values scaled with mass to get forces and torques (a wrench) to output to the TAM using a PI controller.
  - ISMC uses velocity reference for calculations.
  - Hydrostatic compensation uses measured position of pitch and roll for calculations.
  - Adjustment of mass based on measured velocity.

### Plugin Library 
velocity_controllers/integral_sliding_mode_controller

### References
  - Target velocity V<sub>ref</sub>: v<sub>x<sub>ref</sub></sub>, v<sub>y<sub>ref</sub></sub>, v<sub>z<sub>ref</sub></sub>, w<sub>rx<sub>ref</sub></sub>, w<sub>ry<sub>ref</sub></sub>, w<sub>rz<sub>ref</sub></sub> [m/s and rad/s]
  - Measured velocity V<sub>me</sub>: v<sub>x<sub>me</sub></sub>, v<sub>y<sub>me</sub></sub>, v<sub>z<sub>me</sub></sub>, w<sub>rx<sub>me</sub></sub>, w<sub>ry<sub>me</sub></sub>, w<sub>rz<sub>me</sub></sub> [m/s and rad/s]

### Commands
The output of this controller is a wrench with force components F<sub>x</sub>, F<sub>y</sub>, F<sub>z</sub> and torque components T<sub>rx</sub>, T<sub>ry</sub>, T<sub>rz</sub> in N and Nm respectively.

### subscribers
  - integral_sliding_mode_controller/reference [geometry_msgs::msg::Twist]
  - integral_sliding_mode_controller/system_state [geometry_msgs::msg::Twist]

### Publishers
  - integral_sliding_mode_controller/status [control_msgs::msg::MultiDOFStateStamped]

### Parameters 
  - use_external_measured_states: Flag to use velocity measurements obtained from a topic instead of from state interfaces. [bool]
  - enable_parameter_update_without_reactivation: If enabled, the parameters will be dynamically updated while the controller is running. [bool]
  - reference_controller: The prefix of the reference controller to send command to. This can be used to configure command interfaces in chained mode. [string]
  - tf:
    - base_frame: The name of the vehicle base frame. [string]
    - odom_frame: The name of the inertial frame. [string]
  - gains:
    - rho: The sliding mode gain. This adjusts how quickly the controller drives the system to the sliding surface. [double]
    - lambda: The boundary thickness of the tanh function used to attenuate ISMC chatter. [double]
    - Kp: The proportional gains for the controller provided in the order: x, y, z, rx, ry, rz. [double array]
  - hydrodynamics:
    - mass: The mass of the vehicle. [double]
    - moments_of_inertia: The moments of inertia of the vehicle in the order: I<sub>xx</sub>, I<sub>yy</sub>, I<sub>zz</sub>. [double array]
    - added_mass: The added mass coefficients of the vehicle in the order: X<sub>du</sub>, Y<sub>dv</sub>, Z<sub>dw</sub>, K<sub>dp</sub>, M<sub>dq</sub>, N<sub>dr</sub>. [double array]
    - weight: The weight of the vehicle. [double]
    - bouyancy: The buoyancy of the vehicle [double]
    - center_of_buoyancy: The center-of-buoyancy of the vehicle in the order: x, y, z. [double array]
    - center_of_gravity: The center-of-gravity of the vehicle in the order: x, y, z. [double array]
    - linear_damping: The linear damping coefficients of the vehicle in the order: X<sub>u</sub>, Y<sub>v</sub>, Z<sub>w</sub>, K<sub>p</sub>, M<sub>q</sub>, N<sub>r</sub>. [double array]
    - quadratic_damping: The quadratic damping coefficients of the vehicle in the order: X<sub>uu</sub>, Y<sub>vv</sub>, Z<sub>ww</sub>, K<sub>pp</sub>, M<sub>qq</sub>, N<sub>rr</sub>. [double array]
