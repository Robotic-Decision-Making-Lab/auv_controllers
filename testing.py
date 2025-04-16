import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=5, suppress=True, linewidth=200)

# Load the URDF model using a free-flyer as root joint
model = pinocchio.buildModelFromUrdf("output.urdf", pinocchio.JointModelFreeFlyer())

joint_names_to_lock = [
    "alpha_rs1_130_joint",
    "alpha_rs1_139_joint",
    "alpha_axis_a",
    "thruster1_joint",
    "thruster2_joint",
    "thruster3_joint",
    "thruster4_joint",
    "thruster5_joint",
    "thruster6_joint",
    "thruster7_joint",
    "thruster8_joint"
]

# Convert the joint names to joint IDs
joint_ids_to_lock = [
    model.getJointId(name) for name in joint_names_to_lock if model.existJointName(name)
]

q_ref = pinocchio.neutral(model)
model = pinocchio.buildReducedModel(model, joint_ids_to_lock, q_ref)
data = model.createData()

rot = R.random()
q = pinocchio.neutral(model)
q[:3] = np.random.rand(3)
q[3:7] = rot.as_quat().reshape(-1)
# q[-1] = 1.54

print(q)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)
pinocchio.updateFramePlacements(model, data)
pinocchio.computeJointJacobians(model, data, q)

# # Get the index of the base link
joint_name = "alpha_axis_b"
joint_id = model.getJointId(joint_name)
joint_idx = model.joints[joint_id].idx_v

frame_name = "alpha_tcp"
frame_id = model.getFrameId(frame_name)
print(f"Frame ID of {frame_name}: {frame_id}")

J = pinocchio.getFrameJacobian(model, data, model.getFrameId(frame_name), pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
# J = pinocchio.getJointJacobian(model, data, joint_id, pinocchio.ReferenceFrame.LOCAL)

print(f"Jacobian of {joint_name}:")
print(J)
