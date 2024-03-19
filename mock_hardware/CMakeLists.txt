cmake_minimum_required(VERSION 3.8)
project(mock_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  ament_cmake
  rclcpp_lifecycle
  pluginlib
  hardware_interface
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(mock_hardware SHARED
  src/mock_hardware.cpp
)

target_include_directories(mock_hardware
  PUBLIC
    $<INSTALL_INTERFACE:include/mock_hardware>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(mock_hardware PUBLIC cxx_std_17)
target_link_libraries(mock_hardware
  PUBLIC
    ${rclcpp_LIBRARIES}
)

# Use dllexport instead of dllimport
target_compile_definitions(mock_hardware PRIVATE "MOCK_HARDWARE_BUILDING_DLL")
ament_target_dependencies(mock_hardware
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface mock_hardware.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/mock_hardware
)

install(
  TARGETS
    mock_hardware
  EXPORT
    export_mock_hardware
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_mock_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
