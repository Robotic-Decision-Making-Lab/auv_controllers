cmake_minimum_required(VERSION 3.23)
project(end_effector_trajectory_controller)

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

generate_parameter_library(end_effector_trajectory_controller_parameters
  src/end_effector_trajectory_controller_parameters.yaml
)

add_library(end_effector_trajectory_controller SHARED)
target_sources(
    end_effector_trajectory_controller
    PRIVATE src/end_effector_trajectory_controller.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/end_effector_trajectory_controller/end_effector_trajectory_controller.hpp
)
target_compile_features(end_effector_trajectory_controller PUBLIC cxx_std_23)
target_link_libraries(
    end_effector_trajectory_controller
    PUBLIC
        end_effector_trajectory_controller_parameters
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
        ${geometry_msgs_TARGETS}
        ${auv_control_msgs_TARGETS}
        ${lifecycle_msgs_TARGETS}
)

install(
    TARGETS
        end_effector_trajectory_controller
        end_effector_trajectory_controller_parameters
    EXPORT export_end_effector_trajectory_controller
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    FILE_SET HEADERS
)

ament_export_targets(export_end_effector_trajectory_controller HAS_LIBRARY_TARGET)
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
)

ament_package()
