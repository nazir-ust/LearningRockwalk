import gym
import math
import time
import numpy as np
import pybullet as bullet
import matplotlib.pyplot as plt

from rock_walk.resources.cone import Cone
from rock_walk.resources.eef import EndEffector
from rock_walk.resources.plane import Plane
from rock_walk.resources.goal import Goal
from rock_walk.resources.controller import ExpertController

from scipy.spatial.transform import Rotation as R

import pkgutil
# for rendering with bullet.ER_TINY_RENDERER
# egl = pkgutil.get_loader('eglRenderer')
# self._plugin = bullet.loadPlugin(egl.get_filename(), "_eglRendererPlugin")


class RockWalkEnv(gym.Env):

    def __init__(self, bullet_connection, step_freq, frame_skip):

        self._desired_nutation = 10

        self._bullet_connection = bullet_connection
        self._frame_skip = frame_skip
        self._ep_timeout = 2.5
        self._mu_min = 0.2
        self._mu_max = 2.0 #1.0

        action_low = np.array([-0.5, -0.5], dtype=np.float64)
        action_high = np.array([0.5, 0.5], dtype=np.float64)
        self.action_space = gym.spaces.box.Box(low=action_low, high=action_high)

        obs_low = np.array([-5, -5, -5, -10, -10, -10], dtype=np.float64)
        obs_high = np.array([5, 5, 5, 10, 10, 10], dtype=np.float64)
        self.observation_space = gym.spaces.box.Box(low=obs_low, high=obs_high)

        self.bullet_setup(bullet_connection)
        bullet.setTimeStep(1./step_freq, self.clientID)

        self.np_random, _ = gym.utils.seeding.np_random() #input seed eg. 0 for repeatability
        self.reset()


    def step(self, action):
        duration = time.time()-self.start_time
        if duration > self._ep_timeout:
            print("terminated: timout")
            self.done = True

        self.cone.apply_action(action)

        for _ in range(self._frame_skip):
            bullet.stepSimulation()

        true_cone_state, true_cone_te = self.cone.get_observation()

        noisy_cone_state = self.cone.get_noisy_observation(self.np_random)

        reward = self.set_rewards(true_cone_state, true_cone_te)

        if self._bullet_connection == 2:
            self.adjust_camera_pose()

        ob = np.array([noisy_cone_state[2], noisy_cone_state[3], noisy_cone_state[4],
                       noisy_cone_state[7], noisy_cone_state[8], noisy_cone_state[9]], dtype=np.float64)

        return ob, reward, self.done, dict()


    def reset(self):
        self._mu_cone_ground = np.inf #self.np_random.uniform(self._mu_min,self._mu_max)
        self._yaw_spawn = np.pi/2 #self.np_random.uniform(-np.pi, np.pi) #np.pi/2 #+ self.np_random.uniform(-np.pi/4, np.pi/4)

        bullet.resetSimulation(self.clientID)
        bullet.setGravity(0, 0, -9.8)
        self.initialize_physical_objects()
        self.done = False
        self.start_time = time.time()
        if self._bullet_connection == 2:
            self.adjust_camera_pose()

        true_cone_state, true_cone_te = self.cone.get_observation()
        noisy_cone_state = self.cone.get_noisy_observation(self.np_random)

        self.prev_x = [true_cone_state[0]]

        ob = np.array([noisy_cone_state[2], noisy_cone_state[3], noisy_cone_state[4],
                       noisy_cone_state[7], noisy_cone_state[8], noisy_cone_state[9]], dtype=np.float64)

        return ob


    def set_rewards(self, cone_state, cone_te):

        if np.linalg.norm(np.array([cone_state[0], cone_state[1]])) > 15:
            print("terminated: distance exceeded 15 meters")
            self.done=True
            reward = 0

        elif len(bullet.getClosestPoints(self._coneID, self._planeID, 0.02)) == 0:
            print("terminated: object off the ground")
            self.done = True
            reward = -50

        elif cone_state[3]>np.radians(45):
            print("terminated: nutation out of bound")
            self.done=True
            reward = -50


        else:
            # reward = 1000*max(cone_state[0]-self.prev_x[0],0) \
            reward = 20*np.exp(-(min(0, cone_state[0]-self.prev_x[0]))**2) \
                    +10*np.exp(-(min(0, cone_te-5.0))**2) + 10*np.exp(-(max(0, cone_te-5.0))**2)
            self.prev_x = [cone_state[0]]

        return reward


    def initialize_physical_objects(self):
        Goal(self.clientID)
        self.plane = Plane(self.clientID)
        self._planeID = self.plane.get_ids()[0]
        self.plane.set_lateral_friction(self._mu_cone_ground)

        self.cone = Cone(self.clientID, self._yaw_spawn)
        self._coneID = self.cone.get_ids()[0]
        self.cone.set_lateral_friction(self._mu_cone_ground)

    def bullet_setup(self, bullet_connection):

        if bullet_connection == 0:
            self.clientID = bullet.connect(bullet.DIRECT)
            # bullet.setDefaultContactERP(0.9)
        elif bullet_connection == 1:
            self.clientID = bullet.connect(bullet.GUI)
        elif bullet_connection == 2:
            self.clientID = bullet.connect(bullet.SHARED_MEMORY)
            bullet.configureDebugVisualizer(bullet.COV_ENABLE_GUI,0)
            self._cam_dist = 2.5 #3
            self._cam_yaw = -20 + 90
            self._cam_pitch = -45 + 15

    def adjust_camera_pose(self):
        cone_pos_world = bullet.getLinkState(self._coneID,linkIndex=5,physicsClientId=self.clientID)[0]
        bullet.resetDebugVisualizerCamera(cameraDistance=self._cam_dist,
                                          cameraYaw=self._cam_yaw,
                                          cameraPitch=self._cam_pitch,
                                          cameraTargetPosition=cone_pos_world)

    def render(self):
        # if mode == "human":
        # 	self.isRender = True
        # if mode != "rgb_array":
        # 	return np.array([])
        self._cam_dist = 3
        self._cam_yaw = -20
        self._cam_pitch = -30
        self._render_width = 320
        self._render_height = 240

        cone_id, client_id = self.cone.get_ids()
        base_pos, _ = bullet.getBasePositionAndOrientation(cone_id, client_id)

        view_matrix = bullet.computeViewMatrixFromYawPitchRoll(
        	cameraTargetPosition=base_pos,
        	distance=self._cam_dist,
        	yaw=self._cam_yaw,
        	pitch=self._cam_pitch,
        	roll=0,
        	upAxisIndex=2)

        proj_matrix = bullet.computeProjectionMatrixFOV(
        	fov=60, aspect=float(self._render_width)/self._render_height, nearVal=0.1, farVal=100.0)


        (_, _, px, _, _) = bullet.getCameraImage(width = self._render_width,
                                                 height=self._render_height,
                                                 viewMatrix=view_matrix,
                                                 projectionMatrix=proj_matrix,
                                                 renderer=bullet.ER_BULLET_HARDWARE_OPENGL)

        # bullet.configureDebugVisualizer(bullet.COV_ENABLE_SINGLE_STEP_RENDERING,1)
        np_img_arr = np.reshape(px, (self._render_height, self._render_width, 4))
        np_img_arr = np_img_arr * (1. / 255.)

        return np_img_arr


    def close(self):
        bullet.disconnect(self.clientID)


    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]




# self.eef1 = EndEffector(self.clientID, pos=[0.1,-0.25,1.30], orientation=[0,0,np.pi/4])
# self._eef1ID = self.eef1.get_ids()[0]
# self.eef2 = EndEffector(self.clientID, pos=[0.1,0.25,1.30], orientation=[0,0,5*np.pi/4])
# self._eef2ID = self.eef2.get_ids()[0]

# self._eef1_rot = R.from_euler('z', -np.pi/4).as_matrix()
# self._eef2_rot = R.from_euler('z', -5*np.pi/4).as_matrix()


    # self._action_scaling = #self.np_random.uniform(0.1,1)


# elif cone_state[3]<1e-1:
#     print("terminated: cone upright")
#     self.done=True
#     reward = -50

# elif abs(cone_state[4])>np.pi/2:
#     print("terminated: spin out of bound")
#     self.done=True
#     reward = -50


# if self._count_z_action == 1:
#     self.move_eef_z(z_des=action[2], eef=self.eef)
#     self._count_z_action += 1
#     print(action[2])



# self.move_eef_z(z_des=1.2, eef=self.eef)
