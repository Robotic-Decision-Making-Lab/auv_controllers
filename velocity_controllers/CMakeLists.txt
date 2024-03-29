cmake_minimum_required(VERSION 3.8)
project(velocity_controllers)

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
  hydrodynamics
  geometry_msgs
  control_msgs
  tf2_eigen
  tf2
  tf2_ros
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(integral_sliding_mode_controller_parameters
  src/integral_sliding_mode_controller_parameters.yaml
)

add_library(velocity_controllers SHARED
  src/integral_sliding_mode_controller.cpp
)

target_include_directories(velocity_controllers
  PUBLIC
    $<INSTALL_INTERFACE:include/velocity_controllers>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(velocity_controllers PUBLIC cxx_std_17)
target_link_libraries(velocity_controllers
  PUBLIC
    ${rclcpp_LIBRARIES}
    integral_sliding_mode_controller_parameters
)

# Use dllexport instead of dllimport
target_compile_definitions(velocity_controllers PRIVATE "VELOCITY_CONTROLLERS_BUILDING_DLL")
ament_target_dependencies(velocity_controllers
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface velocity_controllers.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/velocity_controllers
)

install(
  TARGETS
    velocity_controllers
    integral_sliding_mode_controller_parameters
  EXPORT
    export_velocity_controllers
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_velocity_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
