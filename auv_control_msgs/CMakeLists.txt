cmake_minimum_required(VERSION 3.8)
project(auv_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(auv_control_msgs
  "msg/MultiActuatorStateStamped.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
