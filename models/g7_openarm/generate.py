import os
import sys

os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))

from symbolic_generator import KinematicsOrientation, SymbolicGenerator


# The generated library contract is expressed in robot joint names, never in
# Pinocchio q/v offsets. The generator resolves these names after loading URDF.
ACTUATED_JOINTS = [
    "AMR_FL_joint",
    "AMR_FLW_joint",
    "AMR_FR_joint",
    "AMR_FRW_joint",
    "AMR_RL_joint",
    "AMR_RLW_joint",
    "AMR_RR_joint",
    "AMR_RRW_joint",
    "L_1_joint",
    "L_2_joint",
    "L_3_joint",
    "L_4_joint",
    "L_5_joint",
    "L_6_joint",
    "L_7_joint",
    "R_1_joint",
    "R_2_joint",
    "R_3_joint",
    "R_4_joint",
    "R_5_joint",
    "R_6_joint",
    "R_7_joint",
]

symb_gen = SymbolicGenerator(
    "g7_openarm.urdf",
    floating=True,
    kinematics_bodies=["L_tcp", "R_tcp"],
    actuated_joints=ACTUATED_JOINTS,
    kinematics_ori=KinematicsOrientation.Quaternion,
    gen_dir="./generated_code/g7_openarm_quat",
)
symb_gen.generate()
