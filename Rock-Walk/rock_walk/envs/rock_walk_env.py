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

import pkgutil
# for rendering with bullet.ER_TINY_RENDERER
# egl = pkgutil.get_loader('eglRenderer')
# self._plugin = bullet.loadPlugin(egl.get_filename(), "_eglRendererPlugin")


class RockWalkEnv(gym.Env):

    def __init__(self, bullet_connection, step_freq, frame_skip):

        self._bullet_connection = bullet_connection
        self._frame_skip = frame_skip
        self._ep_timeout = 5.
        self._mu_min = 0.2
        self._mu_max = 2.0 #1.0

        action_low = np.array([-1, -1], dtype=np.float64)
        action_high = np.array([1, 1], dtype=np.float64)
        self.action_space = gym.spaces.box.Box(low=action_low, high=action_high)

        obs_low = np.array([-5, -5, -5, -10, -10, -10], dtype=np.float64)
        obs_high = np.array([5, 5, 5, 10, 10, 10], dtype=np.float64)
        self.observation_space = gym.spaces.box.Box(low=obs_low, high=obs_high)

        self.bullet_setup(bullet_connection)
        bullet.setTimeStep(1./step_freq, self.clientID)

        self.np_random, _ = gym.utils.seeding.np_random() #input seed eg. 0 for repeatability
        self.reset()


    def step(self, action):
        action = action/self._action_scaling

        duration = time.time()-self.start_time
        if duration > self._ep_timeout:
            print("terminated: timout")
            self.done = True

        self.eef.apply_action(action)

        for _ in range(self._frame_skip):
            bullet.stepSimulation()

        true_eef_state = self.eef.get_observation()
        true_cone_state = self.cone.get_observation()
        noisy_cone_state = self.cone.get_noisy_observation(self.np_random)


        reward = self.set_rewards(true_cone_state, true_eef_state)

        if self._bullet_connection == 2:
            self.adjust_camera_pose()


        ob = np.array([noisy_cone_state[2], noisy_cone_state[3], noisy_cone_state[4],
                       noisy_cone_state[7], noisy_cone_state[8], noisy_cone_state[9]], dtype=np.float64)


        return ob, reward, self.done, dict()


    def reset(self):
        self._mu_cone_ground = 0.5 #self.np_random.uniform(self._mu_min,self._mu_max)
        self._action_scaling = 3 #self.np_random.uniform(1,5)
        self._yaw_spawn = np.pi/2 + self.np_random.uniform(-np.pi/4, np.pi/4)


        bullet.resetSimulation(self.clientID)
        bullet.setGravity(0, 0, -9.8)
        self.initialize_physical_objects()
        self.done = False
        self.start_time = time.time()
        if self._bullet_connection == 2:
            self.adjust_camera_pose()

        true_cone_state = self.cone.get_observation()
        noisy_cone_state = self.cone.get_noisy_observation(self.np_random)

        self.prev_x = [true_cone_state[0]]

        ob = np.array([noisy_cone_state[2], noisy_cone_state[3], noisy_cone_state[4],
                       noisy_cone_state[7], noisy_cone_state[8], noisy_cone_state[9]], dtype=np.float64)

        return ob


    def set_rewards(self, cone_state, eef_state):

        if len(bullet.getClosestPoints(self._coneID, self._eefID, 0.03)) == 0:
            print("terminated: end-effector off the cone")
            self.done = True
            reward = -50

        # elif abs(cone_state[2]-(-np.pi/2)) > np.pi/3:
        #     print("terminated: yaw out of bound")
        #     # self.done=True
        #     reward = -50

        # elif cone_state[3]<np.radians(10):
        #     # print("terminated: cone almost upright")
        #     # self.done=True
        #     reward = -50
        #
        # elif abs(cone_state[4])>np.pi/2:
        #     print("terminated: spin out of bound")
        #     # self.done=True
        #     reward = -50

        elif cone_state[3]>np.radians(5) and abs(cone_state[4])>np.pi/2:
                print("spin out of bound")
                reward = -50


        else:
            reward = 1000*max(cone_state[0]-self.prev_x[0],0)
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

        self.eef = EndEffector(self.clientID, self._yaw_spawn)
        self._eefID = self.eef.get_ids()[0]


        # init_theta = np.radians(25 + self.np_random.uniform(-5,5))
        # self.initial_cone_tilting(theta_des=init_theta)


    def initial_cone_tilting(self, theta_des):
        theta = 0
        # self.eef.speedl([0.5,0.,0.])
        self.eef.speedl([0.5*np.sin(self._yaw_spawn),-0.5*np.cos(self._yaw_spawn),0.])
        while theta < theta_des:
            bullet.stepSimulation()
            cone_state = self.cone.get_observation()
            theta = cone_state[3]
        self.eef.speedl([0.,0.,0.])
        bullet.stepSimulation()
        print("Initial tilting done")



    def move_eef_z(self, z_des, eef):

        curr_pos = eef.get_observation()
        curr_z = curr_pos[2]
        eef.speedl([0.,0.,0.5])
        while curr_z < z_des:
            bullet.stepSimulation()
            curr_pos = eef.get_observation()
            curr_z = curr_pos[2]
            # time.sleep(1./240.)
        eef.speedl([0.,0.,0.])
        bullet.stepSimulation()


    def adjust_camera_pose(self):
        base_pos, _ = bullet.getBasePositionAndOrientation(self._coneID, self.clientID)
        bullet.resetDebugVisualizerCamera(cameraDistance=self._cam_dist,
                                          cameraYaw=self._cam_yaw,
                                          cameraPitch=self._cam_pitch,
                                          cameraTargetPosition=base_pos)

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


    def bullet_setup(self, bullet_connection):

        if bullet_connection == 0:
            self.clientID = bullet.connect(bullet.DIRECT)
            # bullet.setDefaultContactERP(0.9)
        elif bullet_connection == 1:
            self.clientID = bullet.connect(bullet.GUI)
        elif bullet_connection == 2:
            self.clientID = bullet.connect(bullet.SHARED_MEMORY)
            bullet.configureDebugVisualizer(bullet.COV_ENABLE_GUI,0)
            self._cam_dist = 2.0 #3
            self._cam_yaw = -20 +90
            self._cam_pitch = -45


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
