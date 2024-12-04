from matchteris.env import *
import cv2
env = Assemble_Suction(render_mode="rgb")
env.generate_blocks()
for i in range(500):
    env.mocap_ctrl()
rgb,depth = env.get_camera_data("side")
# rgb,depth = env.get_camera_data("eyeinhand")
cv2.imshow("rgb",rgb)
cv2.imshow("depth",depth)
cv2.waitKey(0)


