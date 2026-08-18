import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator, KinematicsOrientation


symb_gen = SymbolicGenerator('g7_openarm.urdf', 
                             floating = True,
                             kinematics_bodies=['L_tcp', 'R_tcp'],
                             actuated_dofs = [
                                *range(6, 21),
                                *range(23, 30),
                             ],
                             kinematics_ori = KinematicsOrientation.Quaternion,
                             gen_dir="./generated_code/g7_openarm_quat")
symb_gen.generate()
