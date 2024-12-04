from matchteris.env import Assemble
env = Assemble()
env.generate_blocks()
while True:
    env.mocap_ctrl()