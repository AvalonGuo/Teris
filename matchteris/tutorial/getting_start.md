# Getting Start

## Project Architecture

### The simulation's files
There are many simulation files ended with xml.Here are three important xml files.
- [ideal_suction](matchteris/env/components/ideal_sucktion.xml)：provides a ideal simulation for sucking teris.
- [ag95_scene](matchteris/env/components/ag95/ag95_scene.xml)：provides a ideal simulation for grasp block.
- [assign_scene](matchteris/env/components/ur5e/assign_scene.xml)：a simulation env including a ur5e robot,a table and a baseplate.

Using the mujoco.viewer,we can quikly preview the mjcf files ended with xml. Now open a terminal and paste this cmd：```python3 -m mujoco.viewer```.Then drag the file into the UI.

### Mujoco viewer
This project uses mujoco.viewer to render the simulation env.Mujoco viewer contains two part：right_ui_tab and left ui tab.
- The right ui tab contains joint and actuactor.You can drag the actuator slider to control the robot.
- The left ui tab contains many parts.We just need to pay attention to  *Rendering's Camera and Label*.

### env directory
This directory contains two environments：[grasp](matchteris/env/assemble_clamp.py)(not recommanded) and [suck](matchteris/env/assemble_suction.py)(recommanded).And the 'components' directory includes some terises and robots.

 ### demo
 This directory's files demostrates how to use the environment.
 - [gtg](matchteris/demo/ground_truth_grasp.py) and [gts](matchteris/demo/ground_truth_suck.py) illustrate how to suck/grasp using ground-truth position.
 - [mocap_control](matchteris/demo/mocap_control.py) illustrate the mocap concept.
 - [use_camera](matchteris/demo/use_camera.py) shows how to get rgb/depth data from the camera.