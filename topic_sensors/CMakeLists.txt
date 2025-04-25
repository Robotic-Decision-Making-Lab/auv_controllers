cmake_minimum_required(VERSION 3.23)
project(topic_sensors)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    rclcpp
    realtime_tools
    std_msgs
    nav_msgs
    controller_common
    message_transforms
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

add_library(topic_sensors SHARED)
target_sources(
    topic_sensors
    PRIVATE src/odom_sensor.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/topic_sensors/odom_sensor.hpp
)
ament_target_dependencies(topic_sensors PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(topic_sensors PUBLIC cxx_std_23)
target_link_libraries(topic_sensors PUBLIC ${rclcpp_LIBRARIES})
pluginlib_export_plugin_description_file(hardware_interface topic_sensors.xml)

install(
    TARGETS topic_sensors
    EXPORT export_topic_sensors
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_topic_sensors HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
