import pybullet as bullet
import numpy as np

import os
from rock_walk.resources.utils import *

# import pybullet_data


class Cone:
    def __init__(self, clientID, yaw_spawn):
        self.clientID = clientID
        self._yaw_spawn = yaw_spawn
        # f_name1 = os.path.join(os.path.dirname(__file__),'models/moai_bm_scaled_down.urdf')
        # f_name3 = os.path.join(os.path.dirname(__file__), 'models/mesh/tex_0.jpg')
        f_name1 = os.path.join(os.path.dirname(__file__),'models/large_cone_apex_control.urdf')

        orientation = bullet.getQuaternionFromEuler([0,0,yaw_spawn],physicsClientId=self.clientID)
        self.coneID = bullet.loadURDF(fileName=f_name1, basePosition=[0, 0, 1.5],
                                      baseOrientation=orientation,#([0,0,np.pi/2]),
                                      useFixedBase=1,
                                      physicsClientId=self.clientID)


        # texID = bullet.loadTexture(f_name3)
        # bullet.changeVisualShape(self.coneID, -1, textureUniqueId=texID, physicsClientId=client)


    def get_ids(self):
        return self.coneID, self.clientID

    def get_joint_info(self, idx):
        print(bullet.getJointInfo(self.coneID, idx, physicsClientId=self.clientID))

    def get_dynamics_info(self):
        print(bullet.getDynamicsInfo(self.coneID, -1, physicsClientId=self.clientID))

    def set_lateral_friction(self, value):
        bullet.changeDynamics(self.coneID, -1, lateralFriction=value, physicsClientId=self.clientID)

    def apply_action(self, action):

        action_rot = R.from_euler('z', -self._yaw_spawn).as_matrix()
        action = np.matmul(action_rot, np.array([action[0], action[1], 0]))



        bullet.setJointMotorControl2(self.coneID,
                                     jointIndex=0,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=action[0],
                                     physicsClientId=self.clientID)

        bullet.setJointMotorControl2(self.coneID,
                                     jointIndex=1,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=action[1],
                                     physicsClientId=self.clientID)

        bullet.setJointMotorControl2(self.coneID,
                                     jointIndex=2,
                                     controlMode=bullet.VELOCITY_CONTROL,
                                     targetVelocity=action[2],
                                     force=0,
                                     physicsClientId=self.clientID)

    def get_observation(self):
        lin_pos_base_world, quat_base_world = bullet.getLinkState(self.coneID,linkIndex=5,physicsClientId=self.clientID)[0:2]
        lin_vel_base_world, ang_vel_base_world = bullet.getLinkState(self.coneID,linkIndex=5,computeLinkVelocity=1,
                                                                     physicsClientId=self.clientID)[-2:]

        # lin_pos_base_world, quat_base_world = bullet.getBasePositionAndOrientation(self.coneID, self.clientID)

        rot_base_world = bullet.getMatrixFromQuaternion(quat_base_world, self.clientID)
        rot_body_world = transform_to_body_frame(rot_base_world)

        psi, theta, phi = compute_body_euler(rot_body_world)

        # lin_vel_base_world, ang_vel_base_world = bullet.getBaseVelocity(self.coneID, self.clientID)

        psi_dot, theta_dot, phi_dot = compute_body_velocity(rot_body_world, ang_vel_base_world)


        state = [lin_pos_base_world[0], lin_pos_base_world[1], psi, theta, phi,
                 lin_vel_base_world[0], lin_vel_base_world[1], psi_dot, theta_dot, phi_dot]


        com_ke = 0.5*1.04*(lin_vel_base_world[0]**2+lin_vel_base_world[1]**2+lin_vel_base_world[2]**2)
        com_pe = 1.04*10*lin_pos_base_world[2]
        rot_ke = compute_rotation_ke(ang_vel_base_world)

        total_energy = com_ke + com_pe + rot_ke

        return state, total_energy

    def get_noisy_observation(self, np_random):

        cone_state, cone_te = self.get_observation()
        # mu = np.zeros([10,])
        return cone_state+np_random.normal(0.0,0.02,10) #0.05


# def change_constraint(self):
#
#     lin_pos_base_world, quat_base_world = bullet.getBasePositionAndOrientation(self.coneID, self.clientID)
#     rot_base_world = bullet.getMatrixFromQuaternion(quat_base_world, self.clientID)
#
#     rot_base_world_np = np.array([[rot_base_world[0], rot_base_world[1], rot_base_world[2]],
#                                   [rot_base_world[3], rot_base_world[4], rot_base_world[5]],
#                                   [rot_base_world[6], rot_base_world[7], rot_base_world[8]]])
#
#     apex_vector = np.matmul(rot_base_world_np, np.array([0.0, -0.262339, 1.109297]))
#
#     apex_pos_base_world = [lin_pos_base_world[0]+apex_vector[0],
#                            lin_pos_base_world[1]+apex_vector[1],
#                            lin_pos_base_world[2]+apex_vector[2]]
#
#     bullet.changeConstraint(self.constraintID, apex_pos_base_world, quat_base_world)




# f_name1 = os.path.join(os.path.dirname(__file__),'models/large_cone_apex_control_a.urdf')
# f_name2 = os.path.join(os.path.dirname(__file__),'models/large_cone_apex_control_b.urdf')

#
# self.coneID = bullet.loadURDF(fileName=f_name1, basePosition=[0, 0, 1.50],
#                               baseOrientation=bullet.getQuaternionFromEuler([0,0,self._yaw_spawn]),#([0,0,np.pi/2]),
#                               physicsClientId=client)
#
# self.coneCtrlID = bullet.loadURDF(fileName=f_name2, basePosition=[0, 0, 1.50],
#                               baseOrientation=bullet.getQuaternionFromEuler([0,0,self._yaw_spawn]),#([0,0,np.pi/2]),
#                               useFixedBase=1,
#                               physicsClientId=client)
#
# self.constraintID = bullet.createConstraint(self.coneCtrlID, 3, self.coneID, -1, bullet.JOINT_POINT2POINT,
#                                             [0, 0, 0],[0, 0, 0], [0.0, -0.262339, 1.109297])
