cmake_minimum_required(VERSION 3.23)
project(impedance_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(controller_common REQUIRED)
find_package(hydrodynamics REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)

generate_parameter_library(impedance_controller_parameters
    src/impedance_controller_parameters.yaml
)

add_library(impedance_controller SHARED)

target_sources(
    impedance_controller
    PRIVATE src/impedance_controller.cpp
    PUBLIC
        FILE_SET HEADERS
            BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/include/impedance_controller/impedance_controller.hpp
)
target_compile_features(impedance_controller PUBLIC cxx_std_23)
target_link_libraries(
    impedance_controller
    PRIVATE
        impedance_controller_parameters
        controller_interface::controller_interface
        hardware_interface::hardware_interface
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        realtime_tools::realtime_tools
        Eigen3::Eigen
        tf2_eigen::tf2_eigen
        tf2::tf2
        tf2_ros::tf2_ros
        controller_common::controller_common
        hydrodynamics::hydrodynamics
        ${geometry_msgs_TARGETS}
        ${control_msgs_TARGETS}
        ${nav_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface impedance_controller.xml)

install(
    TARGETS impedance_controller impedance_controller_parameters
    EXPORT export_impedance_controller
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_impedance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
    "controller_interface"
    "hardware_interface"
    "rclcpp"
    "rclcpp_lifecycle"
    "realtime_tools"
    "Eigen3"
    "geometry_msgs"
    "control_msgs"
    "tf2_eigen"
    "tf2"
    "tf2_ros"
    "nav_msgs"
    "controller_common"
    "hydrodynamics"
)

ament_package()
