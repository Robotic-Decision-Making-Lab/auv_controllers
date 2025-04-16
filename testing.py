import pinocchio
import numpy as np

np.set_printoptions(precision=5, suppress=True, linewidth=200)

# Load the URDF model using a free-flyer as root joint
model = pinocchio.buildModelFromUrdf("output.urdf", pinocchio.JointModelFreeFlyer())

# Define the joint names you wish to "remove" (i.e., lock)
joint_names_to_lock = ["alpha_rs1_130_joint", "alpha_rs1_139_joint", "alpha_axis_a"]

# Convert the joint names to joint IDs
joint_ids_to_lock = []
for name in joint_names_to_lock:
    if model.existJointName(name):
        joint_ids_to_lock.append(model.getJointId(name))
    else:
        print("Warning: Joint '{}' does not exist in the model.".format(name))

# Create a reference configuration for the original model.
# This must have the dimension model.nq.
q_ref = pinocchio.neutral(model)

# Build the reduced model by locking the specified joints.
# The returned value is the reduced model.
reduced_model = pinocchio.buildReducedModel(model, joint_ids_to_lock, q_ref)

# Create a data object for the reduced model
data = reduced_model.createData()

# Compute the neutral configuration for the reduced model
q_neutral = pinocchio.neutral(reduced_model)

# Print out the joint names contained in the reduced model.
for name in reduced_model.names:
    print(name)

print("Neutral configuration for the reduced model:")
print(q_neutral)

# rot = R.random()
# q = pinocchio.neutral(model)
# q[:3] = np.random.rand(3)
# q[3:7] = rot.as_quat().reshape(-1)
# q[-1] = 1.54

# # Perform the forward kinematics over the kinematic tree
# pinocchio.forwardKinematics(model, data, q)
# pinocchio.updateFramePlacements(model, data)
# pinocchio.computeJointJacobians(model, data, q)

# # Get the index of the base link
# joint_name = "axis_d"
# joint_id = model.getJointId(joint_name)
# joint_idx = model.joints[joint_id].idx_v


# print(joint_idx)

# # J_base = pinocchio.getFrameJacobian(model, data, joint_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
# J = pinocchio.getJointJacobian(model, data, joint_id, pinocchio.ReferenceFrame.LOCAL)

# print(f"Jacobian of {joint_name}:")
# print(J)
