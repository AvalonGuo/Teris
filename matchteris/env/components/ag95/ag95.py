from dm_control import mjcf

class AG95:
    def __init__(self) -> None:
        self.mjcf_root = mjcf.from_path("matchteris/env/components/ag95/ag95.xml")

    @property
    def gripper_site_name(self):
        return self.mjcf_root.model+"/gripper_center"
    @property
    def gripper_actuator_name(self):
        return self.mjcf_root.model+"/fingers_actuator"
