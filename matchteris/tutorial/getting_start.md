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
