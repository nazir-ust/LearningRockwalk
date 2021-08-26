import pybullet as bullet
import numpy as np

import os
from rock_walk.resources.utils import *

class EndEffector:
    def __init__(self, client, pos, orientation):
        self.clientID = client
        f_name = os.path.join(os.path.dirname(__file__),'models/end_effector.urdf')


        # base_position = [0,-0.3,1.50] # 1.20

        self.eefID = bullet.loadURDF(fileName=f_name,
                                     basePosition=pos,#[0.35,0.,1.20],#[0.,-0.35,1.20], #1.20 z
                                     baseOrientation=bullet.getQuaternionFromEuler(orientation), #[0,0,np.radians(45)]),
                                     useFixedBase=1,
                                     physicsClientId=client)

    def get_ids(self):
        return self.eefID, self.clientID

    def get_dynamics_info(self):
        print(bullet.getDynamicsInfo(self.eefID, -1, self.clientID))


    def apply_action(self, action):

        bullet.setJointMotorControl2(self.eefID,
                                     jointIndex=0,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=action[0],
                                     physicsClientId=self.clientID)

        bullet.setJointMotorControl2(self.eefID,
                                     jointIndex=1,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=action[1],
                                     physicsClientId=self.clientID)

        # bullet.setJointMotorControl2(self.eefID,
        #                              jointIndex=2,
        #                              controlMode=bullet.VELOCITY_CONTROL,
        #                              targetVelocity=action[2],
        #                              physicsClientId=self.clientID)



    def move_z(self, z_des):
        bullet.setJointMotorControl2(self.eefID,
                                    jointIndex=2,
                                    controlMode=bullet.POSITION_CONTROL,
                                    targetPosition=z_des,
                                    physicsClientId=self.clientID)


    def speedl(self, speed):
        bullet.setJointMotorControl2(self.eefID,
                                     jointIndex=0,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=speed[0],
                                     physicsClientId=self.clientID)

        bullet.setJointMotorControl2(self.eefID,
                                     jointIndex=1,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=speed[1],
                                     physicsClientId=self.clientID)

        bullet.setJointMotorControl2(self.eefID,
                                     jointIndex=2,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=speed[2],
                                     physicsClientId=self.clientID)


    def get_observation(self):

        link_pos_world = bullet.getLinkState(self.eefID,linkIndex=2,physicsClientId=self.clientID)[0]
        link_vel_world = bullet.getLinkState(self.eefID,linkIndex=2,computeLinkVelocity=1,physicsClientId=self.clientID)[-2]

        state = [link_pos_world[0], link_pos_world[1], link_pos_world[2],
                 link_vel_world[0], link_vel_world[1], link_vel_world[2]]

        return state
