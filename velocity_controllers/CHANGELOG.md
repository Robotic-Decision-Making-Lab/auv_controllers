# Changelog for package velocity_controllers

## 0.3.1 (2025-07-09)

## 0.3.0 (2025-06-07)

## 0.2.1 (2025-06-03)

## 0.2.0 (2025-05-03)

## 0.1.0 (2025-04-27)

- Updates the API to use C++ 23
- Bumps the minimum CMake version to CMake 23
- Implements the adaptive integral terminal sliding mode controller
- Changes the integral sliding mode controller external state message interface
from the geometry_msgs/TwistStamped message type to nav_msgs/Odometry
- Makes the integral sliding mode controller joint names configurable for users
that want to integrate prefixes into the command/state interfaces
- Updates the integral sliding mode controller to use the latest hydrodynamics
parsing capabilities
