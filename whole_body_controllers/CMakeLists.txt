cmake_minimum_required(VERSION 3.23)
project(whole_body_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_common
    controller_interface
    control_msgs
    Eigen3
    eigen3_cmake_module
    geometry_msgs
    hardware_interface
    message_transforms
    nav_msgs
    pinocchio
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    tf2_ros
    tf2_eigen
    trajectory_msgs
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

generate_parameter_library(task_priority_solver_parameters
  src/ik_solvers/task_priority_solver_parameters.yaml
)

# ik base solver library
# this needs to be independent of the plugin library
add_library(solver_base SHARED)
target_sources(
    solver_base
    PRIVATE src/ik_solvers/solver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/whole_body_controllers/ik_solvers/solver.hpp
)
ament_target_dependencies(solver_base PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(solver_base PUBLIC cxx_std_23)

# ik solver plugin library
add_library(ik_solvers SHARED)
target_sources(
    ik_solvers
    PRIVATE src/ik_solvers/task_priority_solver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/whole_body_controllers/ik_solvers/task_priority_solver.hpp
)
ament_target_dependencies(ik_solvers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(ik_solvers PUBLIC cxx_std_23)
target_link_libraries(
    ik_solvers
    PUBLIC solver_base task_priority_solver_parameters
)

# the first argument is the name of the package, NOT the target name
pluginlib_export_plugin_description_file(ik_solvers solvers.xml)

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
target_link_libraries(whole_body_controllers PUBLIC ik_controller_parameters ik_solvers)
pluginlib_export_plugin_description_file(controller_interface whole_body_controllers.xml)

install(
    TARGETS whole_body_controllers ik_controller_parameters solver_base ik_solvers task_priority_solver_parameters
    EXPORT export_whole_body_controllers
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_whole_body_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
