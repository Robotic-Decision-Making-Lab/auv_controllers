cmake_minimum_required(VERSION 3.23)
project(thruster_allocation_matrix_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    controller_interface
    hardware_interface
    realtime_tools
    Eigen3
    geometry_msgs
    auv_control_msgs
    control_msgs
    hydrodynamics
    controller_common
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)

generate_parameter_library(thruster_allocation_matrix_controller_parameters
  src/thruster_allocation_matrix_controller_parameters.yaml
)

add_library(thruster_allocation_matrix_controller SHARED)

target_sources(
    thruster_allocation_matrix_controller
    PRIVATE src/thruster_allocation_matrix_controller.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/thruster_allocation_matrix_controller/thruster_allocation_matrix_controller.hpp
)
target_compile_features(thruster_allocation_matrix_controller PUBLIC cxx_std_23)
target_link_libraries(
    thruster_allocation_matrix_controller
    PUBLIC
        thruster_allocation_matrix_controller_parameters
        controller_interface::controller_interface
        hardware_interface::hardware_interface
        hardware_interface::mock_components
        rclcpp::rclcpp
        hydrodynamics::hydrodynamics
        rclcpp_lifecycle::rclcpp_lifecycle
        realtime_tools::realtime_tools
        realtime_tools::thread_priority
        controller_common::controller_common
        ${control_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${auv_control_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface thruster_allocation_matrix_controller.xml)

install(
    TARGETS
        thruster_allocation_matrix_controller
        thruster_allocation_matrix_controller_parameters
    EXPORT export_thruster_allocation_matrix_controller
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_thruster_allocation_matrix_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
