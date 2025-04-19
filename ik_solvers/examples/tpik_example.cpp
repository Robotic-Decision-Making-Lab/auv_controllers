// Copyright 2025, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ik_solvers/solver.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

auto main(int argc, char ** argv) -> int
{
  rclcpp::init(argc, argv);

  // create a new node for the solver
  // the solver needs the node to create parameters
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("solver_node");

  // create a class loader for the solver
  pluginlib::ClassLoader<ik_solvers::IKSolver> loader("ik_solvers", "ik_solvers::IKSolver");

  // load the pinocchio model from the demo URDF file
  // because the AUV has a floating base, we need to use the free flyer joint model
  const std::string urdf_file = "/home/ubuntu/ws_ros/src/auv_controllers/ik_solvers/examples/urdf/example.urdf";
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_file, pinocchio::JointModelFreeFlyer(), *model);

  // the loaded model has several redundant joints that we don't want to use, e.g., the thrusters and TCP
  // we can remove these joints by locking them in the model
  const std::vector<std::string> locked_joints = {
    "thruster1_joint",
    "thruster2_joint",
    "thruster3_joint",
    "thruster4_joint",
    "thruster5_joint",
    "thruster6_joint",
    "thruster7_joint",
    "thruster8_joint",
    "alpha_rs1_130_joint",
    "alpha_rs1_139_joint",
    "alpha_axis_a"};

  std::vector<pinocchio::JointIndex> locked_joint_ids;
  std::ranges::transform(locked_joints, std::back_inserter(locked_joint_ids), [model](const auto & joint) {
    return model->getJointId(joint);
  });

  // build the reduced model using the locked joints
  // we also use the neutral configuration of the model as the initial configuration
  const auto reduced_model = std::make_shared<pinocchio::Model>();
  pinocchio::buildReducedModel(*model, locked_joint_ids, pinocchio::neutral(*model), *reduced_model);

  // create the pinocchio data object from the reduced model
  auto data = std::make_shared<pinocchio::Data>(*reduced_model);

  // create the solver and initialize it
  std::shared_ptr<ik_solvers::IKSolver> solver = loader.createSharedInstance("task_priority_solver");
  solver->initialize(node, reduced_model, data);

  // create an initial configuration for the model
  Eigen::VectorXd q = pinocchio::neutral(*reduced_model);
  q[0] = 1.0;      // base x-position
  q[7] = 3.14159;  // axis-e angle

  // update the model and data with the initial configuration
  pinocchio::forwardKinematics(*reduced_model, *data, q);
  pinocchio::updateFramePlacements(*reduced_model, *data);
  pinocchio::computeJointJacobians(*reduced_model, *data, q);

  // create a target pose for the end effector

  node->shutdown();
  rclcpp::shutdown();

  return 0;
}
