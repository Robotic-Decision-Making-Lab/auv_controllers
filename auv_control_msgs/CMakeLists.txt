cmake_minimum_required(VERSION 3.23)
project(auv_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(auv_control_msgs
  "msg/MultiActuatorStateStamped.msg"
  "msg/IKControllerStateStamped.msg"
  "msg/EndEffectorTrajectoryPoint.msg"
  "msg/EndEffectorTrajectory.msg"
  "msg/EndEffectorTrajectoryControllerState.msg"
  "action/FollowEndEffectorTrajectory.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs trajectory_msgs
)

ament_package()
