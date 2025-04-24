cmake_minimum_required(VERSION 3.23)
project(whole_body_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    nav_msgs
    std_msgs
    control_msgs
    pinocchio
    geometry_msgs
    tf2_ros
    tf2_eigen
    message_transforms
    controller_common
    ik_solvers
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

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
ament_target_dependencies(whole_body_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(whole_body_controllers PUBLIC cxx_std_23)
target_link_libraries(whole_body_controllers PUBLIC ik_controller_parameters)
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
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
