# End Effector Trajectory Controller

The end effector trajectory controller interpolates an end effector motion plan
for whole-body inverse kinematic control. The positions are interpolated using
linear interpolation. The orientations are interpolated using spherical linear
interpolation.

## Plugin Library

geometric_trajectory_controller/GeometricTrajectoryController

## References

The input to this controller is a sequence of end effector poses.

## Commands

The output of this controller is a sampled end effector pose.

## Subscribers

- geometric_trajectory_controller/trajectory [auv_control_msgs::msg::GeometricTrajectory]

## Action Servers

- geometric_trajectory_controller/follow_trajectory [auv_control_msgs::action::FollowGeometricTrajectory]

## Publishers

- geometric_trajectory_controller/status [auv_control_msgs::msg::GeometricTrajectoryControllerState]
