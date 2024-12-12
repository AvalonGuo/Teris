from matchteris.env import *
import numpy as np
env = Assemble_Suction(render_mode="rgb")
target_points = {
    "initial_point":np.asanyarray([-0.415,0,0.8]),
    "move_right":np.asanyarray([-0.415,0.255,0.8]),
    "move_forward":np.asanyarray([-0.785,0.255,0.8]),
    "move_left":np.asanyarray([-0.785,-0.255,0.8]),
    "move_back_a":np.asanyarray([-0.415,-0.255,0.8]),
    "move_back_b":np.asanyarray([-0.215,-0.255,0.8]),
    "move_down":np.asanyarray([-0.215,-0.255,0.41]),
}
eef_maxerror  = np.zeros(3)
for key in ["initial_point", "move_right", "move_forward", "move_left", "move_back_a","move_back_b","move_down"]:
    target = target_points[key]
    env.move_to(target,2)
    eef_curerror = env.get_eff_error()
    eef_maxerror = np.maximum(eef_maxerror,eef_curerror)
print(np.round(eef_maxerror*1000,3))