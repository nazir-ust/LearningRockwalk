import gym
import math
import time
import numpy as np
import pybullet as bullet

import matplotlib.pyplot as plt

# from rock_walk.resources.cone import Cone
from rock_walk.resources.trainingObject import TrainObject
from rock_walk.resources.eef import EndEffector
from rock_walk.resources.plane import Plane
from rock_walk.resources.goal import Goal
from rock_walk.resources.controller import ExpertController

from scipy.spatial.transform import Rotation as R

class RockWalkEnv(gym.Env):

    def __init__(self, bullet_connection, step_freq, frame_skip, isTrain):
        self._bullet_connection = bullet_connection
        self._frame_skip = frame_skip
        self._isTrain = isTrain
        self._episode_timout = 3

        self._desired_nutation = 15 # in degrees

        self._object_param_file_path = "/home/nazir/learning_rockwalk/workstation/training_objects_params.txt"
        if self._isTrain==True:
            self._init_object_param = list(np.loadtxt(self._object_param_file_path, delimiter=',', skiprows=1, dtype=np.float64)[-1:].flatten())
        else:
            ellipse_params=[0.35,0.35]
            apex_coordinates=[0,-0.35,1.5]
            self._init_object_param = ellipse_params + apex_coordinates

        self.bullet_setup(self._bullet_connection)
        bullet.setTimeStep(1./step_freq, self.clientID)

        self.goal = Goal(self.clientID) # for visualizing the direction of transport.
        self.plane = Plane(self.clientID) # support surface representing ground
        self.cone = TrainObject(self.clientID)
        self.cone.generate_object_mesh(ellipse_params=self._init_object_param[:2],
                                       apex_coordinates=self._init_object_param[2:], density=10)
        self.cone.generate_urdf_file()

        action_low = np.array([-1, -1], dtype=np.float64)
        action_high = np.array([1, 1], dtype=np.float64)
        self.action_space = gym.spaces.box.Box(low=action_low, high=action_high)

        obs_low = np.array([-5, -5, -5, -10, -10, -10]+[0,0,-5,-5,-5], dtype=np.float64)
        obs_high = np.array([5, 5, 5, 10, 10, 10]+[5,5,5,5,5], dtype=np.float64)
        self.observation_space = gym.spaces.box.Box(low=obs_low, high=obs_high)

        self.np_random, _ = gym.utils.seeding.np_random() #input seed eg. 0 for repeatability
        self.reset()

    def step(self, action):

        duration = time.time()-self.start_time
        if duration > self._episode_timout:
            print("terminated: timout")
            self.done = True

        if self._isTrain==True:
            data = np.loadtxt(self._object_param_file_path, delimiter=',', skiprows=1, dtype=np.float64)
            object_param = list(data[-1,:])
            action_scale = 0.5 #self._random_action_scale
        else:
            object_param = self._init_object_param
            action_scale = 0.1

        self.cone.apply_action(action*action_scale)

        for _ in range(self._frame_skip):
            bullet.stepSimulation()

        true_cone_state, true_cone_te = self.cone.get_observation()
        noisy_cone_state = self.cone.get_noisy_observation(self.np_random)

        reward = self.set_rewards(true_cone_state, true_cone_te, action*action_scale)

        if self._bullet_connection == 2:
            self.adjust_camera_pose()

        ob = np.array([noisy_cone_state[2], noisy_cone_state[3], noisy_cone_state[4],
                       noisy_cone_state[7], noisy_cone_state[8], noisy_cone_state[9]]+object_param, dtype=np.float64)

        return ob, reward, self.done, dict()


    def reset(self):
        self.done = False
        bullet.resetSimulation(self.clientID)
        bullet.setGravity(0, 0, -9.8)

        if self._isTrain==True:
            object_param = list(np.loadtxt(self._object_param_file_path, delimiter=',', skiprows=1, dtype=np.float64)[-1:].flatten())
            yaw_spawn = np.pi/2 #+ self.np_random.uniform(-np.pi/4, np.pi/4) #self.np_random.uniform(-np.pi, np.pi) #
        else:
            object_param = self._init_object_param
            yaw_spawn = np.pi/2

        mu_cone_ground = np.inf
        self._random_action_scale = self.np_random.uniform(0.1, 1.0)
        self.initialize_physical_objects(yaw_spawn, mu_cone_ground)

        self.start_time = time.time()
        if self._bullet_connection == 2:
            self.adjust_camera_pose()

        true_cone_state, true_cone_te = self.cone.get_observation()
        noisy_cone_state = self.cone.get_noisy_observation(self.np_random)

        self.prev_x = [true_cone_state[0]]
        self.prev_a = [0,0]

        ob = np.array([noisy_cone_state[2], noisy_cone_state[3], noisy_cone_state[4],
                       noisy_cone_state[7], noisy_cone_state[8], noisy_cone_state[9]]+object_param, dtype=np.float64)

        return ob


    def set_rewards(self, cone_state, cone_te, action):

        if len(bullet.getClosestPoints(self._coneID, self._planeID, 0.02)) == 0:
            print("terminated: object off the ground")
            self.done = True
            reward = -50

        else:
            action_accel = np.linalg.norm(np.array([action[0]-self.prev_a[0], action[1]-self.prev_a[1]]))
            reward = 500*(cone_state[0]-self.prev_x[0]) \
                     -50*max(np.radians(10)-cone_state[3],0) \
                     -25*max(cone_state[3]-np.radians(35), 0) \
                     -15*abs(cone_state[4]) \
                     -5*action_accel
            self.prev_x = [cone_state[0]]
            self.prev_a = [action[0], action[1]]


        return reward

    def initialize_physical_objects(self,yaw_spawn,mu_cone_ground):
        self.goal.load_model_from_urdf()

        self.plane.load_model_from_urdf()
        self.plane.load_texutre()
        self.plane.set_lateral_friction(mu_cone_ground)
        self._planeID = self.plane.get_ids()[0]

        self.cone.load_model_from_urdf(yaw_spawn)
        self.cone.set_lateral_friction(mu_cone_ground)
        self._coneID = self.cone.get_ids()[0]

        self._initial_nutation = np.radians(self._desired_nutation) # + self.np_random.uniform(-5,5))

        if self._isTrain==True:
            self.initial_cone_tilting(theta_des=self._initial_nutation)
            pass
        else:
            self.initial_cone_tilting(theta_des=self._initial_nutation)
            pass


    def initial_cone_tilting(self, theta_des):
        theta = 0
        self.cone.apply_action([0.5,0])
        while theta < theta_des:
            bullet.stepSimulation()
            cone_state = self.cone.get_observation()[0]
            theta = cone_state[3]
        self.cone.apply_action([0.,0.])
        bullet.stepSimulation()
        # print("Initial tilting done")


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
        bullet.setPhysicsEngineParameter(enableFileCaching=0)


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



# elif abs(cone_state[3]-self._initial_nutation)>np.radians(10):
#     # print("terminated: nutation out of bound")
#     self.done=True
#     reward = -50

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
