cmake_minimum_required(VERSION 3.8)
project(thruster_allocation_matrix_controller)

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
  eigen3_cmake_module
  Eigen3
  geometry_msgs
  control_msgs
  hydrodynamics
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(thruster_allocation_matrix_controller_parameters
  src/thruster_allocation_matrix_controller_parameters.yaml
)

add_library(thruster_allocation_matrix_controller SHARED
  src/thruster_allocation_matrix_controller.cpp
)

target_include_directories(thruster_allocation_matrix_controller
  PUBLIC
    $<INSTALL_INTERFACE:include/thruster_allocation_matrix_controller>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(thruster_allocation_matrix_controller PUBLIC cxx_std_17)
target_link_libraries(thruster_allocation_matrix_controller
  PUBLIC
    ${rclcpp_LIBRARIES}
    thruster_allocation_matrix_controller_parameters
)

# Use dllexport instead of dllimport
target_compile_definitions(thruster_allocation_matrix_controller PRIVATE "THRUSTER_ALLOCATION_MATRIX_CONTROLLER_BUILDING_DLL")
ament_target_dependencies(thruster_allocation_matrix_controller
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface thruster_allocation_matrix_controller.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)

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
  EXPORT
    export_thruster_allocation_matrix_controller
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_thruster_allocation_matrix_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
