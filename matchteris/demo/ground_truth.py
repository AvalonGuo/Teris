from matchteris.env import Assemble
env = Assemble()
env.generate_blocks()
def suck():
    for i in range(500):
        env.mocap_ctrl()
    block_xpos = env.get_block_xpos("orange0/")
    print(block_xpos)
    block_xpos[2]+=0.009
    print(block_xpos)
    for i in range(500):
        env.data.mocap_pos = block_xpos
        env.mocap_ctrl()
    for i in range(10500):
        env.mocap_ctrl()
suck()