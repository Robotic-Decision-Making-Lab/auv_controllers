cmake_minimum_required(VERSION 3.23)
project(auv_control_demos)

set(THIS_PACKAGE_INCLUDE_DEPENDS rclpy ament_cmake)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_export_dependencies(rclpy)

install(
    DIRECTORY
        individual_controller/launch
        individual_controller/config
        individual_controller/xacro
        chained_controllers/launch
        chained_controllers/config
        chained_controllers/xacro
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
