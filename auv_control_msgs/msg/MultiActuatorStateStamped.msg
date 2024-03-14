std_msgs/Header header

# The joint names for the reference values.
string[] reference_names

# The controller reference values.
float64[] reference

# The names of the actuators.
string[] output_names

# Current output of the controller.
float64[] output

# Time between two consecutive updates/execution of the control law.
float64 time_step
