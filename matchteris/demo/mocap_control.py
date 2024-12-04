from matchteris.env import *
import numpy as np
mode = int(input("Choose a mode:\n1: suction_mode\n2: grasp_mode\nyour choice: "))
if mode == 1:
    env = Assemble_Suction()
else:
    env = Assemble_Clamp()
xyzm_error = np.zeros(3)
for i in range(1000):
    env.mocap_ctrl()
for i in range(10000):
    env.mocap_ctrl()
    error = env.get_eff_error()
    xyzm_error = np.maximum(xyzm_error, error)  

    # 打印每次迭代最大的error值  
    print("Maximum error value: ", xyzm_error) 