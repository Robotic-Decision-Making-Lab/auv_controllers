cmake_minimum_required(VERSION 3.23)
project(ik_solvers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp_lifecycle
    Eigen3
    eigen3_cmake_module
    pinocchio
    pluginlib
    trajectory_msgs
    tf2_eigen
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(task_priority_solver_parameters
  src/task_priority_solver_parameters.yaml
)

# base solver library
# this needs to be independent of the plugin library
add_library(solver_base SHARED)
target_sources(
    solver_base
    PRIVATE src/solver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/ik_solvers/solver.hpp
)
ament_target_dependencies(solver_base PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(solver_base PUBLIC cxx_std_23)

# plugin library
add_library(ik_solvers SHARED)
target_sources(
    ik_solvers
    PRIVATE src/task_priority_solver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/ik_solvers/task_priority_solver.hpp
)
ament_target_dependencies(ik_solvers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(ik_solvers PUBLIC cxx_std_23)
target_link_libraries(
    ik_solvers
    PUBLIC solver_base task_priority_solver_parameters
)

# the first argument is the name of the package, NOT the target name
pluginlib_export_plugin_description_file(ik_solvers solvers.xml)

set(EXAMPLES tpik_demo)
foreach(example IN ITEMS ${EXAMPLES})
    add_executable(${example})
    target_sources(${example} PRIVATE examples/src/${example}.cpp)
    target_link_libraries(${example} PUBLIC ik_solvers)
    set_target_properties(
        ${example}
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/examples
    )
endforeach()

install(
    TARGETS solver_base ik_solvers task_priority_solver_parameters
    EXPORT export_ik_solvers
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    FILE_SET HEADERS
)

# ament_export_include_directories(include)
ament_export_libraries(ik_solvers)
ament_export_targets(export_ik_solvers)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
