import numpy as np
from tracikpy import TracIKSolver

panda_fk_solver = TracIKSolver("/home/benchturtle/cross_embodiment_ws/src/gazebo_env/description/urdf/panda_arm_hand_only_ik.urdf","world","panda_hand")
ee_pose = panda_fk_solver.fk(np.array([-7.412135892082005739e-05, -4.335973262786865234e-01, -2.359377504035364836e-05 ,-2.701174259185791016e+00, 1.270804841624340042e-05, 3.063160657882690430e+00 ,7.409617304801940918e-01]))
print(ee_pose)
ur5e_ik_solver = TracIKSolver("/home/benchturtle/cross_embodiment_ws/src/gazebo_env/description/urdf/ur5e_nvidia_with_gripper_solo_ik.urdf","world","tool0")
qout = ur5e_ik_solver.ik(ee_pose,qinit=np.zeros(ur5e_ik_solver.number_of_joints))
print(qout)
# panda_fk_solver = TracIKSolver("/home/benchturtle/cross_embodiment_ws/src/gazebo_env/description/urdf/panda_arm_hand_only_ik.urdf","world","panda_hand")
# ee_pose = panda_fk_solver.fk(np.zeros(panda_fk_solver.number_of_joints))
# ur5e_ik_solver = TracIKSolver("/home/benchturtle/cross_embodiment_ws/src/gazebo_env/description/urdf/ur5e_nvidia_with_gripper_solo_ik.urdf","world","tool0")
# qout = ur5e_ik_solver.ik(ee_pose,qinit=np.zeros(ur5e_ik_solver.number_of_joints))
# print(qout)

#print(ur5e_ik_solver.fk(np.array([0,0,0,0,0,0])))