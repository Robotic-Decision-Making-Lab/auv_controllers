import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=5, suppress=True, linewidth=200)

# Load the URDF model
model = pinocchio.buildModelFromUrdf("output.urdf", pinocchio.JointModelFreeFlyer())
data = model.createData()

rot = R.random()
q = pinocchio.neutral(model)
q[:3] = np.random.rand(3)
q[3:7] = rot.as_quat().reshape(-1)
q[-1] = 1.54

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)
pinocchio.updateFramePlacements(model, data)
pinocchio.computeJointJacobians(model, data, q)

# Get the index of the base link
joint_name = "axis_d"
joint_id = model.getJointId(joint_name)
joint_idx = model.joints[joint_id].idx_v


print(joint_idx)

# J_base = pinocchio.getFrameJacobian(model, data, joint_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
J = pinocchio.getJointJacobian(model, data, joint_id, pinocchio.ReferenceFrame.LOCAL)

print(f"Jacobian of {joint_name}:")
print(J)
