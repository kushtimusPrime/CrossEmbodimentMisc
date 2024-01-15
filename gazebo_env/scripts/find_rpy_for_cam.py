import numpy as np
from scipy.spatial.transform import Rotation

camera_link_to_real_camera_link = np.array([[0,0,1],
                                            [-1,0,0],
                                            [0,-1,0]])
world_to_real_camera_link = np.array([[-0.89490489,  0.30872887, -0.32222929],
                                      [0.43659211,  0.45624015, -0.77539168],
                                      [-0.09237185, -0.83458458, -0.54308013]])

real_camera_link_to_camera_link = np.linalg.inv(camera_link_to_real_camera_link)

world_to_camera_link = world_to_real_camera_link @ real_camera_link_to_camera_link

print(world_to_camera_link)

rpy_angles = Rotation.from_matrix(world_to_camera_link).as_euler('xyz', degrees=False)

print(rpy_angles)
