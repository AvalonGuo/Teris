from matchteris.env import Assemble_Suction
env = Assemble_Suction()
env.generate_blocks()
def suck():
    for i in range(500):
        env.mocap_ctrl()
    block_xpos = env.get_block_xpos("orange0/")
    block_xpos[2]+=0.0135
    block_xpos[0]-= 0.0045
    block_xpos[1]-= 0.0045
    for i in range(500):
        env.data.mocap_pos = block_xpos
        env.mocap_ctrl()
    for i in range(500):
        env.data.ctrl[-1] = 1
        env.mocap_ctrl()
    block_xpos[-1]+=0.05
    for i in range(1500):
        env.data.mocap_pos = block_xpos
        env.mocap_ctrl()
suck()