# Inverse Kinematics Solvers

This package implements inverse kinematics (IK) solvers for IK
control.

## Closed-Loop Task Priority Inverse Kinematics Solver

A singularity-robust IK solver proposed by Moe, et al. [^1]. The solver
implements joint limit constraints (using the joint limits defined in the robot
description) and end-effector pose tracking. The Jacobian pseudoinverse is
calculated using the damped least squares method.

[^1]: S. Moe, G. Antonelli, A. R. Teel, K. Y. Pettersen, and J. Schrimpf,
"Set-Based Tasks within the Singularity-Robust Multiple Task-Priority Inverse
Kinematics Framework: General Formulation, Stability Analysis, and Experimental
Results", in *Frontiers in Robotics and AI*, 2016.

### Plugin Library

ik_solvers/task_priority_solver
