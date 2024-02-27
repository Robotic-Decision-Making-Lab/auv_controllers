cmake_minimum_required(VERSION 3.8)
project(controller_testing)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclpy
  ament_cmake
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
