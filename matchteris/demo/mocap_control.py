from matchteris.env import *
import numpy as np
mode = int(input("Choose a mode:\n1: suction_mode\n2: grasp_mode\nyour choice: "))
if mode == 1:
    env = Assemble_Suction()
else:
    env = Assemble_Clamp()

while True:
    env.mocap_ctrl()
