cmake_minimum_required(VERSION 3.23)
project(whole_body_controllers)

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
    nav_msgs
    std_msgs
    control_msgs
    Eigen3
    eigen3_cmake_module
    pinocchio
    geometry_msgs
    tf2_ros
    tf2_eigen
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)

# Generate the parameter libraries
generate_parameter_library(task_priority_ik_solver_parameters
  src/solvers/task_priority_ik_solver_parameters.yaml
)
generate_parameter_library(ik_controller_parameters
  src/ik_controller_parameters.yaml
)

# Add the IK solvers
add_library(ik_solvers SHARED)
target_sources(
    ik_solvers
    PRIVATE src/solvers/ik_solver.cpp src/solvers/task_priority_ik_solver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/whole_body_controllers/solvers/ik_solver.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/whole_body_controllers/solvers/task_priority_ik_solver.hpp
)
ament_target_dependencies(ik_solvers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(ik_solvers PUBLIC cxx_std_23)
target_link_libraries(ik_solvers PUBLIC task_priority_ik_solver_parameters)

# Add the IK controller
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
target_link_libraries(
    whole_body_controllers
    PUBLIC ik_solvers ik_controller_parameters
)

pluginlib_export_plugin_description_file(whole_body_controllers ik_solvers.xml)
pluginlib_export_plugin_description_file(controller_interface whole_body_controllers.xml)

install(
    TARGETS
        ik_solvers
        task_priority_ik_solver_parameters
        whole_body_controllers
        ik_controller_parameters
    EXPORT export_whole_body_controllers
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_whole_body_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
