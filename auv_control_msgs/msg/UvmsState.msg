# This is a message that holds data to describe the state of a free-floating manipulator system.
#
# The state of the system is defined by:
#   * the pose of the vehicle, specified in the coordinate frame given by header.frame_id
#   * the twist of the vehicle, specified in the coordinate frame given by header.child_frame_id
#   * the effort of the vehicle, specified in the coordinate frame given by header.child_frame_id
#   * the position of each manipulator joint (rad or m)
#   * the velocity of each manipulator joint (rad/s or m/s)
#   * the effort that is applied in each manipulator joint (Nm or N).
#
# Each field is optional. For example, if the effort for the vehicle is not known, the message
# may be left empty.
#
# Each manipulator joint is uniquely identified by its name. The joint positions, velocities, and
# efforts should be stored using the same order that the names are declared in. All arrays in this
# message should have the same size, or be empty.

Header header
string child_frame_id

geometry_msgs/Pose base_pose
geometry_msgs/Twist base_twist
geometry_msgs/Wrench base_effort

string[] manipulator_joints
float64[] manipulator_positions
float64[] manipulator_velocities
float64[] manipulator_efforts
