from dm_control import mjcf

class Teris:
    def __init__(self,xml_path,name):
        self.mjcf_root = mjcf.from_path(xml_path)
        self.mjcf_root.model = name
