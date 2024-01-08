import numpy as np
import cv2
check_panda_np = np.load('check_ur5.npy',allow_pickle=True)
import pdb
pdb.set_trace()
check_panda_dict = check_panda_np.tolist()
cam_extrinsic_matrix = check_panda_dict['cam_extrinsic_matrix']
cam_intrinsic_matrix = check_panda_dict['cam_intrinsic_matrix']
img0 = check_panda_dict['img_0']
img1 = check_panda_dict['img_1']
img2 = check_panda_dict['img_2']
img3 = check_panda_dict['img_3']
img4 = check_panda_dict['img_4']

cv2.imwrite('/home/benchturtle/ur5img0.png',img0['img'])
cv2.imwrite('/home/benchturtle/ur5img1.png',img1['img'])
cv2.imwrite('/home/benchturtle/ur5img2.png',img2['img'])
cv2.imwrite('/home/benchturtle/ur5img3.png',img3['img'])
cv2.imwrite('/home/benchturtle/ur5img4.png',img4['img'])
imgs = [img0,img1,img2,img3,img4]
import pdb
pdb.set_trace()