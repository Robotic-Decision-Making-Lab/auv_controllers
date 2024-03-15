# Trusters Allocation Matrix (TAM) Controller
Converts velocity/thrust from catesian space to thruster space. This is calculated by taking the psuedo-inverse of the user provided thruster allocation matrix (TAM) and multiplying it by the reference forces: pinv(TAM) x input.

## References
Input to this controller is a wrench with force components F_x, F_y, F_z and torque components T_rx, T_ry, T_rz in N and Nm respectively. 

## Commands
The output of this controller is an N-dimentional vector whose elements represent the thrust associated with each thruster defined in the TAM.

## Subscribers
- thruster_allocation_matrix_controller/reference [geometry_msgs::msg::Wrench]

## Publishers
- thruster_allocation_matrix_controller/status [auv_control_msgs::msg::MultiActuatorStateStamped]

## Parameters
- thrusters: List of thruster names. Should be provided in the same order as the TAM [N-sized string array]
- reference_controllers: List of the reference controllers. Used when setting up command interfaces for chained mode. Should be provided in the same order as the thrusters. [N-sized string array]
- TAM:
  - TAM_x: [N-sized double array]
  - TAM_y: [N-sized double array]
  - TAM_z: [N-sized double array]
  - TAM_rx: [N-sized double array]
  - TAM_ry: [N-sized double array]
  - TAM_rz: [N-sized double array]
