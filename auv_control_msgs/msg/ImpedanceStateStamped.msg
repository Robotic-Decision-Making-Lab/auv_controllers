# This message represents the state of the impedance controller.

std_msgs/Header header

# The configuration setpoint.
geometry_msgs/Pose reference_pose

# The velocity setpoint.
geometry_msgs/Twist reference_twist

# The force/torque feed-forward setpoint.
geometry_msgs/Wrench reference_wrench

# The measured configuration.
geometry_msgs/Pose feedback_pose

# The measured velocity.
geometry_msgs/Twist feedback_twist

# The vector representation of the left-trivialized error.
geometry_msgs/Twist error_pose

# The velocity error defined in the child frame ID.
geometry_msgs/Twist error_twist

# Time between two consecutive updates/execution of the control law.
float64 time_step

# The calculated control output.
geometry_msgs/Wrench output
