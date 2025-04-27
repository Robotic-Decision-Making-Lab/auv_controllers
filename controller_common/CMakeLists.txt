cmake_minimum_required(VERSION 3.23)
project(controller_common)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(controller_common SHARED)
target_sources(
    controller_common
    PRIVATE src/common.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/controller_common/common.hpp
)
target_compile_features(controller_common PUBLIC cxx_std_23)
target_link_libraries(
    controller_common
    PUBLIC rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS}
)

install(
    TARGETS controller_common
    EXPORT export_controller_common
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_controller_common HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs)

ament_package()
