# Trusters Allocation Matrix (TAM) Controller

A chainable controller that converts thrust from Cartesian space to thruster
space. This is calculated by taking the pseudoinverse of the user-provided
thruster allocation matrix (TAM) and multiplying it by the reference forces:
pinv(TAM) x input.

## Plugin Library

thruster_allocation_matrix_controller/ThrusterAllocationMatrixController

## References

The input to this controller is a wrench with force components F<sub>x</sub>,
F<sub>y</sub>, F<sub>z</sub> and torque components T<sub>rx</sub>,
T<sub>ry</sub>, T<sub>rz</sub> in N and Nm, respectively.

## Commands

The output of this controller is an N-dimensional vector whose elements
represent the thrust associated with each thruster defined in the TAM.

## Subscribers

- thruster_allocation_matrix_controller/reference [geometry_msgs::msg::Wrench]

## Publishers

- thruster_allocation_matrix_controller/status [auv_control_msgs::msg::MultiActuatorStateStamped]

## Parameters

- thrusters: List of thruster names. Should be provided in the same order as
  the TAM [N-sized string array]
- reference_controllers: List of the reference controllers. Used when setting
  up command interfaces for chained mode. Should be provided in the same order
  as the thrusters. [N-sized string array]
- TAM:
  - TAM<sub>x</sub>: [N-sized double array]
  - TAM<sub>y</sub>: [N-sized double array]
  - TAM<sub>z</sub>: [N-sized double array]
  - TAM<sub>rx</sub>: [N-sized double array]
  - TAM<sub>ry</sub>: [N-sized double array]
  - TAM<sub>rz</sub>: [N-sized double array]
