import numpy as np
from tracikpy import TracIKSolver

ur5_fk_solver = TracIKSolver("/home/lawrence/cross_embodiment_ws/src/gazebo_env/description/urdf/ur5_ik_real.urdf","base_link","wrist_3_link")
ee_pose = ur5_fk_solver.fk(np.array([-3.34355599084963+np.pi, -1.3984358946429651, 1.723738193511963, -1.8961899916278284, -1.5677741209613245, 4.5127763748168945]))
print(ee_pose)
panda_ur5_ik_solver = TracIKSolver("/home/lawrence/cross_embodiment_ws/src/gazebo_env/description/urdf/panda_ur5_gripper_ik_real.urdf","panda_link0","panda_link8")
qout = panda_ur5_ik_solver.ik(ee_pose,qinit=np.zeros(panda_ur5_ik_solver.number_of_joints))
print(qout)
# panda_fk_solver = TracIKSolver("/home/benchturtle/cross_embodiment_ws/src/gazebo_env/description/urdf/panda_arm_hand_only_ik.urdf","world","panda_hand")
# ee_pose = panda_fk_solver.fk(np.zeros(panda_fk_solver.number_of_joints))
# ur5e_ik_solver = TracIKSolver("/home/benchturtle/cross_embodiment_ws/src/gazebo_env/description/urdf/ur5e_nvidia_with_gripper_solo_ik.urdf","world","tool0")
# qout = ur5e_ik_solver.ik(ee_pose,qinit=np.zeros(ur5e_ik_solver.number_of_joints))
# print(qout)

#print(ur5e_ik_solver.fk(np.array([0,0,0,0,0,0])))