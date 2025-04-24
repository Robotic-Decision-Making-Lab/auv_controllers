import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=5, suppress=True, linewidth=200)

# Load the URDF model using a free-flyer as root joint
model = pinocchio.buildModelFromUrdf("/home/ubuntu/ws_ros/src/auv_controllers/ik_solvers/examples/urdf/uvms.urdf", pinocchio.JointModelFreeFlyer())

controlled_joints = ["universe", "root_joint", "alpha_axis_b", "alpha_axis_c", "alpha_axis_d", "alpha_axis_e"]
locked_joints = [model.getJointId(model.names[idx]) for idx, joint in enumerate(model.joints) if model.names[idx] not in controlled_joints]

q_ref = pinocchio.neutral(model)
model = pinocchio.buildReducedModel(model, locked_joints, q_ref)
data = model.createData()

for i, joint in enumerate(model.names):
    print(model.getJointId(joint), joint)
