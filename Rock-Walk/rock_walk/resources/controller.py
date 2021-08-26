import math
import numpy as np
from scipy.spatial.transform import Rotation as R

import os


class ExpertController:
    def __init__(self, desired_nutation):
        self._desired_nutation = desired_nutation
        self._rockwalk_step = 0

    def control_law(self, cone_state):
        tau_tilt = 0.25

        if abs(cone_state[9]) < 0.3 and cone_state[4] > np.radians(0):
            # print("Left Tilt")
            tilt_dir = 1
            action = self.energy_control(cone_state, tau_tilt, tilt_dir)
            return action

        elif abs(cone_state[9]) < 0.3 and cone_state[4] < np.radians(0):
            # print("Right Tilt")
            tilt_dir = 0
            action = self.energy_control(cone_state, tau_tilt, tilt_dir)
            return action

        elif abs(cone_state[4]) < np.radians(5):
            # print("Nutation Control")
            action = self.nutation_control(cone_state)
            return action

        else:
            return np.array([0,0])


    def energy_control(self, cone_state, tau_tilt, tilt_dir):

        rot_psi = R.from_euler('z', cone_state[2]).as_matrix()
        init_rot = R.from_euler('z', math.pi/2).as_matrix()

        if tilt_dir == 0:
            y_prime_axis = np.matmul(np.matmul(rot_psi, init_rot),np.array([[0],[1],[0]])) #tilt right
        elif tilt_dir == 1:
            y_prime_axis = np.matmul(np.matmul(rot_psi, init_rot),np.array([[0],[-1],[0]])) #tilt left

        action = np.array([tau_tilt*y_prime_axis[0,0], tau_tilt*y_prime_axis[1,0]])
        return action


    def nutation_control(self, cone_state):

        rot_psi = R.from_euler('z', cone_state[2]).as_matrix()
        init_rot = R.from_euler('z', math.pi/2).as_matrix()
        x_prime_axis = np.matmul(np.matmul(rot_psi, init_rot),np.array([[1],[0],[0]]))

        nutation_error = self._desired_nutation-cone_state[3]
        tau_nutation = 2*nutation_error

        action = np.array([tau_nutation*x_prime_axis[0,0], tau_nutation*x_prime_axis[1,0]])
        return action
