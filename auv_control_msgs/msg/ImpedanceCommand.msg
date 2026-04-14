std_msgs/Header header

# The frame ID of the velocity and force/torque setpoints.
string child_frame_id

# The configuration setpoint.
geometry_msgs/Pose pose

# The velocity setpoint.
geometry_msgs/Twist twist

# The force/torque setpoint.
geometry_msgs/Wrench wrench
