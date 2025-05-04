cmake_minimum_required(VERSION 3.23)
project(whole_body_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_transforms REQUIRED)
find_package(controller_common REQUIRED)
find_package(ik_solvers REQUIRED)
find_package(auv_control_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)

generate_parameter_library(ik_controller_parameters
  src/ik_controller_parameters.yaml
)

add_library(whole_body_controllers SHARED)
target_sources(
    whole_body_controllers
    PRIVATE src/ik_controller.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/whole_body_controllers/ik_controller.hpp
)
target_compile_features(whole_body_controllers PUBLIC cxx_std_23)
target_link_libraries(
    whole_body_controllers
    PUBLIC
        ik_controller_parameters
        controller_interface::controller_interface
        hardware_interface::hardware_interface
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        realtime_tools::realtime_tools
        pinocchio::pinocchio
        tf2_ros::tf2_ros
        ik_solvers::ik_solvers
        tf2_eigen::tf2_eigen
        message_transforms::message_transforms
        controller_common::controller_common
        Eigen3::Eigen
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${control_msgs_TARGETS}
        ${auv_control_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(controller_interface whole_body_controllers.xml)

install(
    TARGETS whole_body_controllers ik_controller_parameters
    EXPORT export_whole_body_controllers
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_whole_body_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
    "controller_interface"
    "hardware_interface"
    "rclcpp"
    "rclcpp_lifecycle"
    "realtime_tools"
    "nav_msgs"
    "std_msgs"
    "control_msgs"
    "pinocchio"
    "geometry_msgs"
    "tf2_ros"
    "tf2_eigen"
    "message_transforms"
    "controller_common"
    "ik_solvers"
    "auv_control_msgs"
    "Eigen3"
)

ament_package()
