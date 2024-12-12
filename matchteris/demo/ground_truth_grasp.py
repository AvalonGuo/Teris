from matchteris.env import Assemble_Clamp
env = Assemble_Clamp()
env.generate_blocks()
def grasp():
    for i in range(500):
        env.mocap_ctrl()
    block_xpos = env.get_block_xpos("orange0/")
    block_xpos[2]+=0.023
    block_xpos[0]-= 0.0045
    block_xpos[1]-= 0.0045
    for i in range(500):
        env.data.mocap_pos = block_xpos
        env.mocap_ctrl()
    for i in range(500):
        env.data.ctrl[-1] = 0.65
        env.mocap_ctrl()
    block_xpos[-1]+=0.05
    for i in range(1500):
        env.data.mocap_pos = block_xpos
        env.mocap_ctrl()
def move_to():
    target_pos = [-0.4,0,0.45]
    for i in range(1500):
        env.data.mocap_pos = target_pos
        env.mocap_ctrl()
    target_pos[1]+=0.2
    for i in range(150000):
        env.data.mocap_pos = target_pos
        env.mocap_ctrl()
grasp()
move_to()