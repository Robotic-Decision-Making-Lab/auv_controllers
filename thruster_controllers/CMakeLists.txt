cmake_minimum_required(VERSION 3.8)
project(thruster_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    control_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)

generate_parameter_library(polynomial_thrust_curve_controller_parameters
  src/polynomial_thrust_curve_controller_parameters.yaml
)

add_library(thruster_controllers SHARED)

target_sources(
    thruster_controllers
    PRIVATE src/polynomial_thrust_curve_controller.cpp
)
target_include_directories(
    thruster_controllers
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    thruster_controllers
    PUBLIC ${rclcpp_LIBRARIES} polynomial_thrust_curve_controller_parameters
)
ament_target_dependencies(thruster_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(thruster_controllers PUBLIC cxx_std_20)

pluginlib_export_plugin_description_file(controller_interface thruster_controllers.xml)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)

    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/ DESTINATION include/thruster_controllers)

install(
    TARGETS thruster_controllers polynomial_thrust_curve_controller_parameters
    EXPORT export_thruster_controllers
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_export_targets(export_thruster_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
