cmake_minimum_required(VERSION 3.8)
project(thruster_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  control_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(polynomial_thrust_curve_controller_parameters
  src/polynomial_thrust_curve_controller_parameters.yaml
)

add_library(thruster_controllers SHARED
  src/polynomial_thrust_curve_controller.cpp
)

target_include_directories(thruster_controllers
  PUBLIC
    $<INSTALL_INTERFACE:include/thruster_controllers>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(thruster_controllers PUBLIC cxx_std_17)
target_link_libraries(thruster_controllers
  PUBLIC
    ${rclcpp_LIBRARIES}
    polynomial_thrust_curve_controller_parameters
)

# Use dllexport instead of dllimport
target_compile_definitions(thruster_controllers PRIVATE "THRUSTER_CONTROLLERS_BUILDING_DLL")
ament_target_dependencies(thruster_controllers
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface thruster_controllers.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/thruster_controllers
)

install(
  TARGETS
    thruster_controllers
    polynomial_thrust_curve_controller_parameters
  EXPORT
    export_thruster_controllers
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_thruster_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
