
from dm_control import mjcf

class UR5E_Scene:
    def __init__(self):
        self.mjcf_root = mjcf.from_path("matchteris/env/components/ur5e/assign_scene.xml")
        self._attachment_site = self.mjcf_root.find("site",self.attachment_site_name)
    @property
    def eef_site_name(self):
        return "eef_site"
    @property
    def attachment_site_name(self):
        return "attachment_site"
    @property
    def link_names(self):
        return ["shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link"]
    @property
    def joint_names(self):
        return ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
    @property
    def actuator_names(self):
        return ["shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"]
    


    def attach_tool(self,child, pos: list = [0, 0, 0], quat: list = [1, 0, 0, 0]) -> mjcf.Element:
        frame = self._attachment_site.attach(child)
        frame.pos = pos
        frame.quat = quat
        return frame
    def attach_block(self, child,  pos: list = [0, 0, 0], quat: list = [1, 0, 0, 0]) -> mjcf.Element:
        frame = self.mjcf_root.attach(child)
        frame.add('joint',name="block_x",armature="0",damping="0.0005",limited="true",pos=[0,0,0],axis=[1,0,0],stiffness="0",range=[-5.0,5.0],type="slide")
        frame.add('joint',name="block_y",armature="0",damping="0.0005",limited="true",pos=[0,0,0],axis=[0,1,0],stiffness="0",range=[-5.0,5.0],type="slide")
        frame.add('joint',name="block_z",armature="0",damping="0.0005",limited="true",pos=[0,0,0],axis=[0,0,1],stiffness="0",range=[-2.0,2.0],type="slide")
        frame.add('joint',name="block_rot",armature="0",damping="0.0005",pos=[0,0,0],stiffness="0",type="ball")
        frame.pos = pos
        frame.quat = quat

        return frame