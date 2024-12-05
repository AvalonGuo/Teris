from matchteris.env import *
import cv2
import os
env = Assemble_Suction(render_mode="human")
env.generate_blocks()
for i in range(500):
    env.mocap_ctrl()
block_xpos = env.get_block_xpos("orange1/")
block_xpos[2]+=0.0235
block_xpos[0]+=0.07
for i in range(1000):
    env.data.mocap_pos = block_xpos
    env.mocap_ctrl()
rgb,depth = env.get_camera_data("eyeinhand")
cv2.imshow("rgb",rgb)
cv2.imshow("depth",depth)
cv2.waitKey(0)

template_path = os.path.join('matchteris','demo','orange.png')
template = cv2.imread(template_path)
template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
target2match = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
retval = cv2.matchShapes(template_gray,target2match,1,0)
print(retval)
