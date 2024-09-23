cmake_minimum_required(VERSION 3.8)
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
    eigen3_cmake_module
    Eigen3
    geometry_msgs
    auv_control_msgs
    control_msgs
    hydrodynamics
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
)
target_include_directories(
    thruster_allocation_matrix_controller
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    thruster_allocation_matrix_controller
    PRIVATE thruster_allocation_matrix_controller_parameters
)
ament_target_dependencies(thruster_allocation_matrix_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(thruster_allocation_matrix_controller PUBLIC cxx_std_20)

pluginlib_export_plugin_description_file(controller_interface thruster_allocation_matrix_controller.xml)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)

    ament_lint_auto_find_test_dependencies()
endif()

install(
    DIRECTORY include/
    DESTINATION include/thruster_allocation_matrix_controller
)

install(
    TARGETS
        thruster_allocation_matrix_controller
        thruster_allocation_matrix_controller_parameters
    EXPORT export_thruster_allocation_matrix_controller
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_export_targets(export_thruster_allocation_matrix_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
