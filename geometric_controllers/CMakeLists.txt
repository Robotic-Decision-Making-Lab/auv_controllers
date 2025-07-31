cmake_minimum_required(VERSION 3.23)
project(geometric_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(auv_control_msgs REQUIRED)
find_package(controller_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

generate_parameter_library(geometric_trajectory_controller_parameters
  src/geometric_trajectory_controller_parameters.yaml
)

add_library(geometric_controllers SHARED)
target_sources(
    geometric_controllers
    PRIVATE src/trajectory_controller.cpp src/trajectory.cpp
    PUBLIC
        FILE_SET HEADERS
            BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/include/geometric_controllers/trajectory_controller.hpp
                ${CMAKE_CURRENT_SOURCE_DIR}/include/geometric_controllers/trajectory.hpp
)
target_compile_features(geometric_controllers PUBLIC cxx_std_23)
target_link_libraries(
    geometric_controllers
    PUBLIC
        geometric_trajectory_controller_parameters
        realtime_tools::realtime_tools
        controller_common::controller_common
        hardware_interface::hardware_interface
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        tf2_ros::tf2_ros
        tf2_eigen::tf2_eigen
        tf2::tf2
        controller_interface::controller_interface
        rclcpp_action::rclcpp_action
        tf2_geometry_msgs::tf2_geometry_msgs
        ${geometry_msgs_TARGETS}
        ${auv_control_msgs_TARGETS}
        ${lifecycle_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface geometric_controllers.xml)

install(
    TARGETS
        geometric_controllers
        geometric_trajectory_controller_parameters
    EXPORT export_geometric_controllers
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    FILE_SET HEADERS
)

ament_export_targets(export_geometric_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "geometry_msgs"
  "realtime_tools"
  "auv_control_msgs"
  "controller_common"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "hardware_interface"
  "rclcpp"
  "rclcpp_lifecycle"
  "controller_interface"
  "rclcpp_action"
  "lifecycle_msgs"
  "tf2_geometry_msgs"
)

ament_package()
