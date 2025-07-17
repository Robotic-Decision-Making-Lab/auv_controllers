std_msgs/Header header

# The reference sample from the trajectory. This is the sample from the current time instance and used
# for error calculation.
geometry_msgs/Pose reference

# The current end effector state.
geometry_msgs/Pose feedback

# The squared Euclidean norm of the geodesic distance between the reference state and end effector state.
float64 error

# The trajectory controller command. This is the trajectory sample from the next time instance.
geometry_msgs/Pose output
