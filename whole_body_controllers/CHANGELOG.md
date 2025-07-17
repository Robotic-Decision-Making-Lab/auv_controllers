# Changelog for package whole_body_controllers

## 0.3.1 (2025-07-09)

## 0.3.0 (2025-06-07)

## 0.2.1 (2025-06-03)

## 0.2.0 (2025-05-03)

- Replaces instances of `Eigen::Affine3d` with `Eigen::Isometry3d`
- Fixes a bug in the ik_controller reference interfaces where the values sent
  to the reference interfaces themselves (i.e., not as a message) were not
  being transformed into the appropriate coordinate frame for Pinocchio.

## 0.1.0 (2025-04-27)

- Implements an IK controller for controlling a UVMS with a single manipulator
