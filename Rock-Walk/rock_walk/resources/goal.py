import pybullet as bullet
import os
import numpy as np


class Goal:
    def __init__(self, client):
        self.clientID = client

        f_name = os.path.join(os.path.dirname(__file__), 'models/goal_direction.urdf')

        self.goalID = bullet.loadURDF(fileName=f_name, basePosition=[7.5, 0.0, -0.04], physicsClientId=self.clientID)
        
        # self.goalID = bullet.loadURDF(fileName=f_name, basePosition=[0.0, -7.5, -0.04],
        #                               baseOrientation = bullet.getQuaternionFromEuler([0,0,np.pi/2]),
        #                               physicsClientId=self.clientID)
