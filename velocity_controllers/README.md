# Velocity Controllers

This package provides a collection of velocity controllers, which calculate
control commands (e.g., a wrench) needed to track a velocity reference
signal.

## Adaptive Integral Terminal Sliding Mode Controller

A chainable controller designed by Gonzalez-Garcia and Castaneda [^1] that
provides robustness to disturbances using a dynamically adapted disturbance
rejection torques.

> [!NOTE]
> The integral sliding mode controller requires that the AUV hydrodynamic
> parameters be included in the robot description file using the format defined
> by the [hydrodynamics](https://github.com/Robotic-Decision-Making-Lab/hydrodynamics/blob/main/examples/description/bluerov2.model.urdf)
> library.

[^1]: A. Gonzalez-Garcia and H. Castaneda, "Adaptive Integral Terminal Sliding
Mode Control for an Unmanned Surface Vehicle Against External Disturbances", in
*IFAC-PapersOnline*, 2021.

### Plugin Library

velocity_controllers/adaptive_integral_terminal_sliding_mode_controller

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

- adaptive_integral_terminal_sliding_mode_controller/reference [geometry_msgs::msg::Twist]
- adaptive_integral_terminal_sliding_mode_controller/system_state [nav_msgs::msg::Odometry]
- robot_description [std_msgs::msg::String]

### Publishers

- adaptive_integral_terminal_sliding_mode_controller/status [control_msgs::msg::MultiDOFStateStamped]

## Integral Sliding Mode Controller

A chainable dynamic controller designed by Palmer et al. [^2] that provides
robustness to matched uncertainties and attenuates chatter.

> [!NOTE]
> The integral sliding mode controller requires that the AUV hydrodynamic
> parameters be included in the robot description file using the format defined
> by the [hydrodynamics](https://github.com/Robotic-Decision-Making-Lab/hydrodynamics/blob/main/examples/description/bluerov2.model.urdf)
> library.

[^2]: E. Palmer, C. Holm, and G. Hollinger, "Angler: An Autonomy Framework for
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
- integral_sliding_mode_controller/system_state [nav_msgs::msg::Odometry]
- robot_description [std_msgs::msg::String]

### Publishers

- integral_sliding_mode_controller/status [control_msgs::msg::MultiDOFStateStamped]
