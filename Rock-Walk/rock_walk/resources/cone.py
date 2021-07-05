import pybullet as bullet
import numpy as np

import os
from rock_walk.resources.utils import *

# import pybullet_data


class Cone:
    def __init__(self, client, yaw_spawn):
        self.clientID = client
        f_name = os.path.join(os.path.dirname(__file__),
                              'models/large_cone.urdf')


        self.coneID = bullet.loadURDF(fileName=f_name, basePosition=[0, 0, 0],
                                      baseOrientation=bullet.getQuaternionFromEuler([0,0,yaw_spawn]),#([0,0,np.pi/2]),
                                      physicsClientId=client)


    def get_ids(self):
        return self.coneID, self.clientID


    def get_dynamics_info(self):
        print(bullet.getDynamicsInfo(self.coneID, -1, physicsClientId=self.clientID))

    def set_lateral_friction(self, value):
        bullet.changeDynamics(self.coneID, -1, lateralFriction=value, physicsClientId=self.clientID)


    def get_observation(self):

        lin_pos_base_world, quat_base_world = bullet.getBasePositionAndOrientation(self.coneID, self.clientID)
        rot_base_world = bullet.getMatrixFromQuaternion(quat_base_world, self.clientID)
        rot_body_world = transform_to_body_frame(rot_base_world)

        psi, theta, phi = compute_body_euler(rot_body_world)

        lin_vel_base_world, ang_vel_base_world = bullet.getBaseVelocity(self.coneID, self.clientID)

        psi_dot, theta_dot, phi_dot = compute_body_velocity(rot_body_world, ang_vel_base_world)


        state = [lin_pos_base_world[0], lin_pos_base_world[1], psi, theta, phi,
                 lin_vel_base_world[0], lin_vel_base_world[1], psi_dot, theta_dot, phi_dot]


        com_ke = 0.5*1.04*(lin_vel_base_world[0]**2+lin_vel_base_world[1]**2+lin_vel_base_world[2]**2)
        com_pe = 1.04*10*lin_pos_base_world[2]
        rot_ke = compute_rotation_ke(ang_vel_base_world)

        cone_te = [com_ke+com_pe+rot_ke]

        return state


    def get_noisy_observation(self, np_random):

        cone_state = self.get_observation()
        # mu = np.zeros([10,])
        return self.get_observation()+np_random.normal(0.0,0.02,10) #0.05
