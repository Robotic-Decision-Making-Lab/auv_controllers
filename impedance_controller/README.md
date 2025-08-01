# Impedance Controller

A chainable impedance controller. Given an AUV pose $g$ with velocity $\nu$,
the implemented control law is given as follows

```math
\tau = \tau_{\text{ref}} + \textbf{K}_\text{p}(g_\text{ref}^\top g)^{\vee} + \textbf{K}_\text{d}(\nu_\text{ref} - \nu)
```

where $\textbf{K}_\text{p}$ is the desired stiffness, and $\textbf{K}_\text{d}$ is the desired damping.

This control law is commonly used as an inner-loop controller in an MPC
framework. [^1] [^2]

[^1]: I. Dadiotis, A. Laurenzi, and N. Tsagarakis. "Whole-body MPC for highly redundant legged manipulators: experimental evaluation with a 37 DoF dual-arm quadruped," in *IEEE International Conference on Humanoid Robots (Humanoids)*, 2023.
[^2]: J. -P. Sleiman, F. Farshidian, M. V. Minniti and M. Hutter, "A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation," in *IEEE Robotics and Automation Letters*, vol. 6, no. 3, pp. 4688-4695, July 2021.

## Plugin Library

impedance_controller/ImpedanceController

## References

* Target pose $g_\text{ref}$
* Target velocity $\nu_\text{ref}$
* Target force/torque $\tau_{\text{ref}}$

## State Feedback

* Measured pose $g$
* Measured velocity $\nu$

## Commands

The output of this controller are the desired forces/torques $\tau$

## Subscribers

* impedance_controller/reference [auv_control_msgs::msg::ImpedanceCommand]

## Publishers

* impedance_controller/status [control_msgs::msg::MultiDOFStateStamped]
