cmake_minimum_required(VERSION 3.23)
project(ik_solvers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    control_msgs
    Eigen3
    eigen3_cmake_module
    pinocchio
    geometry_msgs
    pluginlib
    tf2_eigen
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(task_priority_ik_solver_parameters
  src/task_priority_ik_solver_parameters.yaml
)

add_library(ik_solvers SHARED)
target_sources(
    ik_solvers
    PRIVATE src/ik_solver.cpp src/task_priority_ik_solver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/ik_solvers/ik_solver.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/ik_solvers/task_priority_ik_solver.hpp
)
ament_target_dependencies(ik_solvers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(ik_solvers PUBLIC cxx_std_23)
# target_compile_definitions(ik_solvers PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(ik_solvers PUBLIC task_priority_ik_solver_parameters)
pluginlib_export_plugin_description_file(ik_solvers ik_solvers.xml)

install(
    TARGETS ik_solvers task_priority_ik_solver_parameters
    EXPORT export_ik_solvers
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_include_directories(include)
ament_export_targets(export_ik_solvers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
