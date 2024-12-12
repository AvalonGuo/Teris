# Matchteris
Matchteris(this repo) is a simulation environment designed for [ROS机器人俄罗斯方块](https://www.annisen.com/newsinfo/6991379.html).This repo is also designed for Jinan University Robot operating system course with education purpose.
## Project's Feature
***·simple robotic arm's control using differential ik controll***

***·supports two types end effector(vaccum gripper and ag95 gripper)***

***·has two cameras with different perspectives***

## TODO
- [ ] fixes the block's unrealistic deformation.
- [ ] implements some simple match algorithm.
- [x] improves the ik algorithmn,reduces the end-effector's position error.

## :fire: Update
* __2024.12.12 fixed the IK problem by adding gravity compensation manually. Now, the eef max position error is $[\pm 0.029,\pm 0.014,\pm 0.157](unit \ mm)$ caculated by [this file](/matchteris/demo/caculate_xyzerror.py).__
* __2024.12.12 added move_to() fuction .__
 

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

## References
*  __Differential Inverse Kinematics algorithm modified from [mjctrl] repository(https://github.com/kevinzakka/mjctrl).__
