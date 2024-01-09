import numpy as np

camera_link_to_real_camera_link = np.array([[0,0,1],
                                            [-1,0,0],
                                            [0,-1,0]])
world_to_real_camera_link = np.array([[0.05701638,  0.63329212, -0.77180971],
                                      [0.99733922, -0.07130471,  0.01516949],
                                      [-0.04542695, -0.770621  , -0.63567261]])

real_camera_link_to_camera_link = np.linalg.inv(camera_link_to_real_camera_link)

world_to_camera_link = world_to_real_camera_link @ real_camera_link_to_camera_link

print(world_to_camera_link)