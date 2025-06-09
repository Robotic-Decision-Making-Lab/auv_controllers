cmake_minimum_required(VERSION 3.23)
project(controller_coordinator)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(rclcpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(controller_coordinator_parameters src/coordinator_parameters.yaml)

add_executable(controller_coordinator)
target_sources(controller_coordinator PRIVATE src/coordinator.cpp)

target_compile_features(controller_coordinator PUBLIC cxx_std_20)
target_link_libraries(
    controller_coordinator
    PUBLIC
        controller_coordinator_parameters
        rclcpp::rclcpp
        ${controller_manager_msgs_TARGETS}
        ${std_srvs_TARGETS}
)

install(
    TARGETS
        controller_coordinator
        controller_coordinator_parameters
        DESTINATION
        lib/controller_coordinator
)

ament_package()
