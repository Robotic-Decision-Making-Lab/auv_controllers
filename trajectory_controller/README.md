# Trajectory Controller

The trajectory controller interpolates an SE(3) motion plan for pose control.
The positions are interpolated using linear interpolation, and the orientations
are interpolated using spherical linear interpolation.

## Plugin Library

trajectory_controller/TrajectoryController

## References

The input to this controller is a sequence of poses.

## Commands

The output of this controller is a sampled pose.

## Subscribers

- trajectory_controller/trajectory [auv_control_msgs::msg::Trajectory]

## Action Servers

- trajectory_controller/follow_trajectory [auv_control_msgs::action::FollowTrajectory]

## Publishers

- trajectory_controller/status [auv_control_msgs::msg::TrajectoryControllerState]
