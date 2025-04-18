cmake_minimum_required(VERSION 3.23)
project(solver_testing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ik_solvers REQUIRED)

add_executable(solver_test)
target_sources(solver_test PRIVATE src/solver_node.cpp)
target_compile_features(solver_test PUBLIC cxx_std_23)
ament_target_dependencies(solver_test PUBLIC pluginlib ik_solvers)

install(
  TARGETS solver_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
