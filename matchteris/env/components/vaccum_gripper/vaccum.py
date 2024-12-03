import os
from dm_control import mjcf

class Vaccum:
    def __init__(self):
        path = os.path.join("matchteris","env","components","vaccum_gripper","vaccum_gripper.xml")
        self.mjcf_root = mjcf.from_path(path)
        self.vaccum_site_name = self.mjcf_root.model + "/vaccum_site"





