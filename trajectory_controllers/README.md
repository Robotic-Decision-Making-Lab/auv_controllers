# Trajectory Controllers

This package provides a collection of trajectory controllers, which interpolate
a nominal trajectory for tracking by an inner-loop controller.

## Cartesian Trajectory Controller

A cartesian trajectory controller that interpolates SE(3) motion plans for
cartesian control. The positions are interpolated using linear interpolation,
and the orientations are interpolated using spherical linear interpolation.

### Plugin Library

trajectory_controller/TrajectoryController

### References

The input to this controller is a sequence of SE(3) poses.

### Commands

The output of this controller is a sampled pose.

### Subscribers

- cartesian_trajectory_controller/trajectory [auv_control_msgs::msg::CartesianTrajectory]

### Action Servers

- cartesian_trajectory_controller/follow_trajectory [auv_control_msgs::action::FollowCartesianTrajectory]

### Publishers

- cartesian_trajectory_controller/status [auv_control_msgs::msg::CartesianTrajectoryControllerState]
