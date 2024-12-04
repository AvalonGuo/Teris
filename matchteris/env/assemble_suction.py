import os
import cv2
import time
import numpy as np
import mujoco
import mujoco.viewer
from dm_control import mjcf
from matchteris.env import *

class Assemble_Suction:
    def __init__(self,render_mode="human"):

        self._Ur5e = UR5E_Scene()
        self._vaccum = Vaccum()
        self._Ur5e.attach_tool(self._vaccum.mjcf_root)

        self._physics = mjcf.Physics.from_mjcf_model(self._Ur5e.mjcf_root)
        self.model = self._physics.model.ptr
        self.data = self._physics.data.ptr
        self._physics = mjcf.Physics.from_mjcf_model(self._Ur5e.mjcf_root)
        self.model = self._physics.model.ptr
        self.data = self._physics.data.ptr

        #Robotic's Control
        self.attachment_site_id = self.model.site(self._Ur5e.attachment_site_name).id
        self.vaccum_site_id = self.model.site(self._vaccum.vaccum_site_name).id
        self.link_ids = [self.model.body(link_name).id for link_name in self._Ur5e.link_names]
        self.jnt_ids = [self.model.joint(joint_name).id for joint_name in self._Ur5e.joint_names]
        self.act_ids = [self.model.actuator(actuator_name).id for actuator_name in self._Ur5e.actuator_names]
        self.robot_nv = len(self.jnt_ids)
        self.mocap_id = self.model.body("target").mocapid[0]
        self.damping = 1e-4
        self.max_angvel = 30
        self.integration_dt = 1.0

        #Mujoco Render
        self.viewer = None
        self.render_mode = render_mode
        self.render_width = 640
        self.render_height = 640
        self.image_renderer = mujoco.Renderer(self.model,self.render_height,self.render_width)
        self._timestep = self.model.opt.timestep

        self.teris_dir = "matchteris/env/components/teris"
        self.teris_list = [f for f in os.listdir(self.teris_dir) if f.endswith(".xml")]
        self.teris_mjcfs = []
        self.teris_handles = []

    def step_simulation(self,jnt_angles=None):
        if self.viewer == None and self.render_mode == "human":
            self.viewer = mujoco.viewer.launch_passive(model=self.model,data=self.data,show_left_ui=True,show_right_ui=True)
        step_start = time.time()
        self.data.ctrl[:6] = jnt_angles[:6]
        # self.data.qpos[self.jnt_ids] = [0,-np.pi/2,-np.pi/2,-np.pi/2,np.pi/2,0]
        mujoco.mj_step(self.model,self.data)
        if self.viewer != None:
            self.viewer.sync()
        time_until_next_step = self._timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    def mocap_ctrl(self):
        # Pre-allocate numpy arrays.
        jac = np.zeros((6, self.model.nv))
        diag = self.damping * np.eye(6)
        error = np.zeros(6)
        error_pos = error[:3]
        error_ori = error[3:]
        site_quat = np.zeros(4)
        site_quat_conj = np.zeros(4)
        error_quat = np.zeros(4)
        # Position error.
        error_pos[:] = self.data.mocap_pos[self.mocap_id] - self.data.site(self.vaccum_site_id).xpos
        # Orientation error.
        mujoco.mju_mat2Quat(site_quat, self.data.site(self.vaccum_site_id).xmat)
        mujoco.mju_negQuat(site_quat_conj, site_quat)
        mujoco.mju_mulQuat(error_quat, self.data.mocap_quat[self.mocap_id], site_quat_conj)
        mujoco.mju_quat2Vel(error_ori, error_quat, 1.0)
        # Get the Jacobian with respect to the end-effector site.
        mujoco.mj_jacSite(self.model, self.data, jac[:3], jac[3:], self.vaccum_site_id)

        # Solve system of equations: J @ dq = error.
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, error)
        # Scale down joint velocities if they exceed maximum.
        if self.max_angvel > 0:
            dq_abs_max = np.abs(dq).max()
            if dq_abs_max > self.max_angvel:
                dq *= self.max_angvel / dq_abs_max

        # Integrate joint velocities to obtain joint positions.
        q = self.data.qpos.copy()
        mujoco.mj_integratePos(self.model, q, dq, self.integration_dt)

        # Set the control signal.
        q = q[:self.robot_nv*2]
        jnt_range_min = self.model.jnt_range.T[0][:self.robot_nv*2]
        jnt_range_max = self.model.jnt_range.T[1][:self.robot_nv*2]

        np.clip(q,jnt_range_min,jnt_range_max, out=q)
        q = q[self.jnt_ids]
        self.step_simulation(jnt_angles=q)

    def rotate_mocap(self,degree):
        degree = np.clip(degree,0,180)
        theta = np.deg2rad(degree)
        rotate_xmat = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        mocap_quat = self.data.mocap_quat[self.mocap_id]
        mocap_xmat = np.zeros(9)
        mujoco.mju_quat2Mat(mocap_xmat,mocap_quat)
        mocap_xmat = mocap_xmat.reshape(3,3)
        mocap_xmat = (rotate_xmat@mocap_xmat).flatten()
        mujoco.mju_mat2Quat(mocap_quat,mocap_xmat)
        self.data.mocap_quat[self.mocap_id] = mocap_quat

    def reload_simulation(self):
        if self.viewer != None:
            self.viewer.close()
            self.viewer = None
        self.image_renderer.close()
        self._physics.reload_from_mjcf_model(self._Ur5e.mjcf_root)
        self.model = self._physics.model.ptr
        self.data = self._physics.data.ptr
        self.data.qpos[self.jnt_ids] = [-1.24,-1.37,-1.53,-1.84,-4.71,0.328]
        self.image_renderer = mujoco.Renderer(self.model,self.render_height,self.render_width)
        self.data.mocap_pos[self.mocap_id] = [-0.415,0,0.8]
        self.data.mocap_quat[self.mocap_id] = [0.0006,0.7074,0.7068,-0.0006]
        self.data.ctrl[-1] = 0

    def generate_blocks(self):
        for teris in self.teris_mjcfs:
            teris.mjcf_root.detach()
        self.teris_mjcfs = []
        self.teris_handles = []
        attach_pos = [-0.755,-0.285,0.45]
        offset = 0.08
        for teris_type in self.teris_list:
            if teris_type != self.teris_list[0]:
                attach_pos[0] += offset
            attach_pos[1] = -0.285
            for i in range(5):
                attach_pos[1]+= offset
                curr_teris_file = os.path.join(self.teris_dir,teris_type)
                curr_teris_name = teris_type.split(".")[0]+str(i)
                teris = Teris(curr_teris_file,curr_teris_name)
                self.teris_mjcfs.append(teris)
                self._Ur5e.attach_block(teris.mjcf_root,pos=attach_pos)
        self.reload_simulation()

    def get_block_xpos(self,name):
        block_id = self.model.body(name).id
        block_xpos = self.data.xpos[block_id]
        return list(block_xpos)
    
    def get_camera_data(self,camera_name:str="side"):
        self.image_renderer.enable_depth_rendering()
        self.image_renderer.update_scene(self.data,camera=camera_name)
        depth_data = self.image_renderer.render()
        self.image_renderer.disable_depth_rendering()
        rgb_data = self.image_renderer.render()
        rgb_data = cv2.cvtColor(rgb_data,cv2.COLOR_BGR2RGB)
        return np.asarray(rgb_data),np.asarray(depth_data)
    
    def get_camera_matrix(self,camera_name:str="side"):
        camera_matrix = {"translation":None,"rotation":None,"focal":None,"image":None}
        camera_id = self.model.camera(camera_name).id
        pos = self.data.cam_xpos[camera_id]
        rot = self.data.cam_xmat[camera_id].reshape(3, 3).T
        fov = self.model.cam_fovy[camera_id]
        # Translation matrix (4x4).
        translation = np.eye(4)
        translation[0:3, 3] = -pos
        # Rotation matrix (4x4).
        rotation = np.eye(4)
        rotation[0:3, 0:3] = rot
        # Focal transformation matrix (3x4).
        focal_scaling = (1./np.tan(np.deg2rad(fov)/2)) * self.render_height / 2.0
        focal = np.diag([-focal_scaling, focal_scaling, 1.0, 0])[0:3, :]
        # Image matrix (3x3).
        image = np.eye(3)
        image[0, 2] = (self.render_width - 1) / 2.0
        image[1, 2] = (self.render_height - 1) / 2.0
        camera_matrix["translation"] = translation
        camera_matrix["rotation"] = rotation
        camera_matrix["focal"] = focal
        camera_matrix["image"] = image
        return camera_matrix
    
    def get_camera_intrinsics(self,camera_name:str="side"):
        camera_id = self.model.camera(camera_name).id
        fovy = self.model.cam_fovy[camera_id]
        theta = np.deg2rad(fovy)
        fx = self.render_width / 2 / np.tan(theta / 2)
        fy = self.render_height / 2 / np.tan(theta / 2)
        cx = (self.render_width - 1) / 2.0
        cy = (self.render_height - 1) / 2.0
        cam_intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        return cam_intrinsics
    
    def get_eff_error(self):
        error_pos = np.zeros(3)
        error_pos[:] = self.data.mocap_pos[self.mocap_id] - self.data.site(self.vaccum_site_id).xpos
        return error_pos

