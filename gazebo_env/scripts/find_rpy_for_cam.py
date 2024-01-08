import numpy as np

camera_link_to_real_camera_link = np.array([[0,0,1],
                                            [-1,0,0],
                                            [0,-1,0]])
world_to_real_camera_link = np.array([[-0.09565235,  0.58821708, -0.80302634],
                                      [0.99493654,  0.03149183, -0.09544393],
                                      [-0.03085298, -0.80808968, -0.58825093]])

real_camera_link_to_camera_link = np.linalg.inv(camera_link_to_real_camera_link)

world_to_camera_link = world_to_real_camera_link @ real_camera_link_to_camera_link

print(world_to_camera_link)