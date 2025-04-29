std_msgs/Header header

# The name of the IK solver used by the controller.
string solver_name

# Position DoF names, e.g., joint or axis names.
string[] position_joint_names

# Velocity DoF names.
string[] velocity_joint_names

# The reference end effector pose.
geometry_msgs/Pose reference

# Time between two consecutive updates/execution of the control law.
# This is often used for integration of the solution to determine the desired positions.
float64 time_step

# The IK solver solution.
trajectory_msgs/JointTrajectoryPoint solution
