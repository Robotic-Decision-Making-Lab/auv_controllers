#include <pluginlib/class_loader.hpp>

#include "ik_solvers/solver.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

auto main(int argc, char ** argv) -> int
{
  rclcpp::init(argc, argv);

  pluginlib::ClassLoader<ik_solvers::IKSolver> loader("ik_solvers", "ik_solvers::IKSolver");

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("solver_node");
  const std::string urdf_file = "/home/ubuntu/ws_ros/src/auv_controllers/output.urdf";

  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_file, pinocchio::JointModelFreeFlyer(), *model);
  auto data = std::make_shared<pinocchio::Data>(*model);

  try {
    std::shared_ptr<ik_solvers::IKSolver> solver = loader.createSharedInstance("task_priority_solver");
    solver->initialize(node, model, data);
  }
  catch (pluginlib::PluginlibException & ex) {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  rclcpp::shutdown();

  return 0;
}
