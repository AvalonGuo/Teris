# Matchteris
Matchteris(this repo) is a simulation environment designed for [ROS机器人俄罗斯方块](https://www.annisen.com/newsinfo/6991379.html).This repo is also designed for Jinan University Robot operating system course with education purpose.
## Project's Feature
***·simple robotic arm's control using differential ik/operational space controll***

***·supports two types end effector(vaccum gripper and ag95 gripper)***

***·has two cameras with different perspectives***

## TODO
- [ ] fixes the block's unrealistic deformation.
- [ ] implements some simple match algorithm.
- [ ] improves the ik algorithmn,reduces the end-effector's position error.

## [Getting start](matchteris/tutorial/getting_start.md)
### Install the package
```
git clone https://github.com/AvalonGuo/Teris.git
cd Teris
pip install -e.
```
### Project Architecture
- [demo](matchteris/demo)：illustrates how to create an env and render it.
- [env](matchteris/env): includes a component directory and two type environments(clamp/suction).
- [tutorial](matchteris/tutorial/getting_start.md)：the tutorial for mujoco and this project.
