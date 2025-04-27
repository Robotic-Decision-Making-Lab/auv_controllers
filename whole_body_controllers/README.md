# Whole Body Controllers

This package provides a collection of whole-body controllers for underwater
vehicle manipulator systems (UVMS).

## Inverse Kinematics Controller

A chainable controller that uses inverse kinematics solvers to compute joint
states required to achieve a desired end effector pose. This controller
can be used with UVMS that have a single manipulator.

### Plugin Library

whole_body_controllers/ik_controller

### References

- Target end effector pose $\eta$<sub>EE</sub>

### State Feedback

- Measured system velocity V: V<sub>vehicle</sub>, V<sub>manipulator</sub>
- Measured system position q: q<sub>vehicle</sub>, q<sub>manipulator</sub>

### Commands

The output of this controller is the solution to the chosen inverse kinematics
solver.

### Subscribers

- ik_controller/reference [geometry_msgs::msg::Pose]
- ik_controller/vehicle_state [nav_msgs::msg::Odometry]

> [!NOTE]
> The IK controller does not currently support sending manipulator states via
> a topic interface.

### Publishers

- ik_controller/status [auv_control_msgs::msg::IKControllerStateStamped]
