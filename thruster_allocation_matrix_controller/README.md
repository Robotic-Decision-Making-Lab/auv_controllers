# Trusters Allocation Matrix (TAM) Controller

A chainable controller that converts thrust from Cartesian space to thruster
space. This is calculated by taking the pseudoinverse of the user-provided
thruster allocation matrix (TAM) and multiplying it by the reference forces:
pinv(TAM) x input.

## Plugin Library

thruster_allocation_matrix_controller/ThrusterAllocationMatrixController

## References

The input to this controller is a wrench with force components $\text{F}_\text{x}$, $F_\text{y}$,
$F_\text{z}$, and torque components $T_\text{rx}$, $T_\text{ry}$, $T_\text{rz}$ in N and Nm, respectively.

## Commands

The output of this controller is an N-dimensional vector whose elements
represent the thrust associated with each thruster defined in the TAM.

## Subscribers

- thruster_allocation_matrix_controller/reference [geometry_msgs::msg::Wrench]

## Publishers

- thruster_allocation_matrix_controller/status [auv_control_msgs::msg::MultiActuatorStateStamped]
