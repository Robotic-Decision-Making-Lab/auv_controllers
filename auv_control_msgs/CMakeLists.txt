cmake_minimum_required(VERSION 3.8)
project(auv_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(auv_control_msgs
  "msg/MultiActuatorStateStamped.msg"
<<<<<<< HEAD
  DEPENDENCIES builtin_interfaces std_msgs
=======
  "msg/IKControllerStateStamped.msg"
  "msg/EndEffectorTrajectoryPoint.msg"
  "msg/EndEffectorTrajectory.msg"
  "msg/EndEffectorTrajectoryControllerState.msg"
  "action/FollowEndEffectorTrajectory.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs trajectory_msgs
>>>>>>> d197fc7 (Port end effector trajectory controller from Angler (#54))
)

ament_package()
