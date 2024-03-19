# Velocity Controllers

This package provides a collection of velocity controllers, which calculate
control commands (e.g., a wrench) needed to track a velocity reference
signal.

## Integral Sliding Mode Controller

A chainable dynamic controller designed by Palmer et al. [^1] that provides
robustness to matched uncertainties and attenuates chatter.

[^1]: E. Palmer, C. Holm, and G. Hollinger, "Angler: An Autonomy Framework for
Intervention Tasks with Lightweight Underwater Vehicle Manipulator Systems," in
*IEEE International Conference on Robotics and Automation (ICRA)*, 2024.

### Plugin Library

velocity_controllers/integral_sliding_mode_controller

### References

- Target velocity V<sub>ref</sub>: v<sub>x<sub>ref</sub></sub>,
  v<sub>y<sub>ref</sub></sub>, v<sub>z<sub>ref</sub></sub>,
  w<sub>rx<sub>ref</sub></sub>, w<sub>ry<sub>ref</sub></sub>,
  w<sub>rz<sub>ref</sub></sub> [m/s and rad/s]

### State Feedback

- Measured velocity V: v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>,
  w<sub>rx</sub>, w<sub>ry</sub>, w<sub>rz</sub> [m/s and rad/s]
- Vehicle orientation measured with respect to the inertial frame Q:
  q<sub>x</sub>, q<sub>y</sub>, q<sub>z</sub>, q<sub>w</sub>

### Commands

The output of this controller is a wrench with force components F<sub>x</sub>,
F<sub>y</sub>, F<sub>z</sub> and torque components T<sub>rx</sub>,
T<sub>ry</sub>, T<sub>rz</sub> in N and Nm, respectively.

### Subscribers

- integral_sliding_mode_controller/reference [geometry_msgs::msg::Twist]
- integral_sliding_mode_controller/system_state [geometry_msgs::msg::Twist]

### Publishers

- integral_sliding_mode_controller/status [control_msgs::msg::MultiDOFStateStamped]

### Parameters

- use_external_measured_states: Flag to use velocity measurements obtained from
  a topic instead of from state interfaces. [bool]
- enable_parameter_update_without_reactivation: If enabled, the parameters will
  be dynamically updated while the controller is running. [bool]
- reference_controller: The prefix of the reference controller to send command
  to. This can be used to configure command interfaces in chained mode. [string]
- tf:
  - base_frame: The name of the vehicle base frame. [string]
  - odom_frame: The name of the inertial frame. [string]
- gains:
  - rho: The sliding mode gain. This adjusts how quickly the controller drives
    the system to the sliding surface. [double]
  - lambda: The boundary thickness of the tanh function used to attenuate ISMC
    chatter. [double]
  - Kp: The proportional gains for the controller provided in the order: x, y,
    z, rx, ry, rz. [double array]
- hydrodynamics:
  - mass: The mass of the vehicle. [double]
  - moments_of_inertia: The moments of inertia of the vehicle in the order:
    I<sub>xx</sub>, I<sub>yy</sub>, I<sub>zz</sub>. [double array]
  - added_mass: The added mass coefficients of the vehicle in the order:
    X<sub>du</sub>, Y<sub>dv</sub>, Z<sub>dw</sub>, K<sub>dp</sub>,
    M<sub>dq</sub>, N<sub>dr</sub>. [double array]
  - weight: The weight of the vehicle. [double]
  - buoyancy: The buoyancy of the vehicle [double]
  - center_of_buoyancy: The center-of-buoyancy of the vehicle in the order: x,
    y, z. [double array]
  - center_of_gravity: The center-of-gravity of the vehicle in the order: x, y,
    z. [double array]
  - linear_damping: The linear damping coefficients of the vehicle in the order:
    X<sub>u</sub>, Y<sub>v</sub>, Z<sub>w</sub>, K<sub>p</sub>, M<sub>q</sub>,
    N<sub>r</sub>. [double array]
  - quadratic_damping: The quadratic damping coefficients of the vehicle in the
    order: X<sub>uu</sub>, Y<sub>vv</sub>, Z<sub>ww</sub>, K<sub>pp</sub>,
    M<sub>qq</sub>, N<sub>rr</sub>. [double array]
