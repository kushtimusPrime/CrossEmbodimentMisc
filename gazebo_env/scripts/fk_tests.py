import numpy as np
from tracikpy import TracIKSolver

ik_solver = TracIKSolver("../description/urdf/robotiq_2f_85_gripper.urdf","world","tool0")
print(np.zeros(ik_solver.number_of_joints))
print(ik_solver.fk(np.array([0.5,-0.5,1,0,0,0])))