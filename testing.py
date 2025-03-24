import pinocchio
from pinocchio.visualize import MeshcatVisualizer
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Transform, Vector3
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=3, suppress=True, linewidth=200)

# Load the URDF model
model = pinocchio.buildModelFromUrdf("output.urdf", pinocchio.JointModelFreeFlyer())
data = model.createData()

rot = R.random()
q = pinocchio.neutral(model)
q[:3] = np.random.rand(3)
q[3:7] = rot.as_quat().reshape(-1)


print(q)

joint_id = model.getJointId("base_footprint_joint")

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)

# Get the position of the joint
joint_position = data.joints[joint_id - 1].joint_q
print(joint_position)

J= pinocchio.computeJointJacobians(model, data, q)
# print(J)
base_link_name = "base_link"
base_link_idx = model.getFrameId(base_link_name)

# Compute the frame Jacobian
J_base = pinocchio.getFrameJacobian(model, data, base_link_idx, pinocchio.ReferenceFrame.WORLD)
# Print the Jacobian
print("Jacobian of base_link:")
print(J_base)
