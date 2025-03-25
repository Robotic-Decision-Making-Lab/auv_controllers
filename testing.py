import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=3, suppress=True, linewidth=200)

# Load the URDF model
model = pinocchio.buildModelFromUrdf("output.urdf", pinocchio.JointModelFreeFlyer())
data = model.createData()

rot = R.random()
q = pinocchio.neutral(model)
q[:3] = np.random.rand(3)
q[3:7] = rot.as_quat().reshape(-1)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)
pinocchio.updateFramePlacements(model, data)

base_link_name = "base_footprint"
base_link_idx = model.getFrameId(base_link_name)
base_joint_idx = model.frames[base_link_idx].parent

print(rot.as_matrix())
print(q)
print(data.oMf[base_link_idx])

J_base = pinocchio.getFrameJacobian(model, data, base_link_idx, pinocchio.ReferenceFrame.WORLD)

print("Jacobian of {base_link_name}:")
print(J_base)
