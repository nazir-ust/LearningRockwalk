import numpy as np
import math
import time
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib import cm

from stable_baselines3 import TD3, SAC, PPO

class LoadModel:
    def __init__(self, filename):
        self.load_model(filename)

    def load_model(self, filename):
        self._model = SAC.load(filename, device="cpu")

    def predict_action(self):
        radius=[0.35]
        apex_coordinates=[-0.35,1.5]
        object_param = radius + apex_coordinates

        steps = 50
        action_1 = np.zeros([steps,steps])
        action_2 = np.zeros([steps,steps])

        phi_range = np.linspace(-np.pi, np.pi, steps)
        phi_dot_range = np.linspace(-2, 2, steps)

        for idx_phi, phi in np.ndenumerate(phi_range):
            for idx_phi_dot, phi_dot in np.ndenumerate(phi_dot_range):
                obs = np.array([-np.pi/2,np.radians(25),phi,0,0,phi_dot])
                action_b, _states = self._model.predict(obs, deterministic=True)

                action_1[idx_phi_dot, idx_phi] = action_b[0]
                action_2[idx_phi_dot, idx_phi] = action_b[1]

        X_phi, Y_phi_dot = np.meshgrid(phi_range, phi_dot_range)

        fig, (ax1, ax2) = plt.subplots(1,2, subplot_kw={"projection": "3d"})
        fig.set_size_inches(18.5, 10.5)
        ax1.plot_surface(X_phi, Y_phi_dot, action_1, cmap=cm.coolwarm)
        ax1.set_xlabel('phi')
        ax1.set_ylabel('phi_dot')
        ax1.set_zlabel('Forward/Backward tilting action')
        ax2.plot_surface(X_phi, Y_phi_dot, action_2, cmap=cm.coolwarm)
        ax2.set_xlabel('phi')
        ax2.set_ylabel('phi_dot')
        ax2.set_zlabel('tilting action')
        ax2.set_title('Sideways tilting action predicted by the learnt policy (theta=25deg)')

        plt.show()



class LoadData:
    def __init__(self, filename):
        # self.plot_action_data()
        self.load_all_data(filename)

    def load_all_data(self, filename):
        data = np.loadtxt(filename, delimiter=',', skiprows=1, dtype=np.float64)

        self._time = data[:,0]
        self._time = self._time-self._time[0]

        self._obs_x = data[:,1]
        self._obs_y = data[:,2]
        self._obs_psi = data[:,3]
        self._obs_theta = data[:,4]
        self._obs_phi = data[:,5]
        self._obs_x_dot = data[:,6]
        self._obs_y_dot = data[:,7]
        self._obs_psi_dot = data[:,8]
        self._obs_theta_dot = data[:,9]
        self._obs_phi_dot = data[:,10]

        self._cone_energy = data[:,11]

        self._action_x = data[:,12]
        self._action_y = data[:,13]

    def plot_main(self):
        # Create two subplots and unpack the output array immediately
        fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, sharex=True)
        fig.set_size_inches(18.5, 10.5)
        ax1.plot(self._time, self._obs_x, label='x_CM')
        ax1.plot(self._time, self._obs_y, label='y_CM')
        ax1.legend()
        ax1.set_title('Position of cone center of mass in meters')

        ax2.plot(self._time, self._obs_theta, label='nutation')
        ax2.plot(self._time, np.radians(25)*np.ones([np.size(self._time)]), label='initial nutation')
        ax2.set_ylim(0, np.pi/2)
        ax2.legend()
        ax2.set_title('Nutation angle in radians')

        ax3.plot(self._time, self._obs_phi)
        ax3.set_ylim(-np.pi, np.pi)
        ax3.set_title('Spin angle in radians')

        ax4.plot(self._time, self._cone_energy, label='Energy of CoM')
        ax4.set_ylim(3.5,5.5)
        ax4.set_title('Cone energy')

        ax5.plot(self._time, 0.25*self._action_x, label='V_1_C')
        ax5.plot(self._time, 0.25*self._action_y, label='V_2_C')
        # ax5.plot(self._time, np.linalg.norm(np.vstack((0.25*self._action_x, 0.25*self._action_y)), axis=0), label='norm')
        ax5.legend()
        ax5.set_title('Actions returned by agent: Velocity of control point long x- and y-axes in meters per second **Actual magnitude is scaled**')
        ax5.set_xlabel('Time (s)')
        plt.show()




def main():

    data = LoadData("./test_data/data.txt")

    data.plot_main()

    load_model = LoadModel("./save/rw_model_1500000_steps")
    load_model.predict_action()



if __name__ == "__main__":
    main()
