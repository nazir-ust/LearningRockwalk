import numpy as np
import math
import time
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure


class LoadData:
    def __init__(self, filename):
        # self.plot_action_data()
        self.load_all_data(filename)


    def load_all_data(self, filename):
        data = np.load(filename)
        self._obs_x = data['obs_x']
        self._obs_y = data['obs_y']
        self._obs_psi = data['obs_psi']
        self._obs_theta = data['obs_theta']
        self._obs_phi = data['obs_phi']
        self._obs_x_dot = data['obs_x_dot']
        self._obs_y_dot = data['obs_y_dot']
        self._obs_psi_dot = data['obs_psi_dot']
        self._obs_theta_dot = data['obs_theta_dot']
        self._obs_phi_dot = data['obs_phi_dot']

        self._action_x = data['action_x']
        self._action_y = data['action_y']
        self._time = data['time']
        self._time = self._time-self._time[0]


    def plot_action_data(self):
        plt.figure()
        plt.plot(self._time, self._action_x)
        plt.plot(self._time, self._action_y)
        plt.show()


def main():

    data = LoadData("./sim_data/data.npz")

    # Create two subplots and unpack the output array immediately
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)
    fig.set_size_inches(18.5, 10.5)
    ax1.plot(data._time, data._obs_x, label='x_CM')
    ax1.plot(data._time, data._obs_y, label='y_CM')
    ax1.legend()
    ax1.set_title('Position of cone center of mass in meters')

    ax2.plot(data._time, data._obs_theta)
    ax2.set_ylim(0, np.pi/2)
    ax2.set_title('Nutation angle in radians')

    ax3.plot(data._time, data._obs_phi)
    ax3.set_ylim(-np.pi, np.pi)
    ax3.set_title('Spin angle in radians')


    ax4.plot(data._time, data._action_x, label='Vx_C')
    ax4.plot(data._time, data._action_y, label='Vy_C')
    ax4.legend()
    ax4.set_title('Velocity of control point (i.e apex of the cone) along x- and y-axes in meters per second')
    ax4.set_xlabel('Time (s)')
    plt.show()
    # plt.show()
    # plt.figure()
    # plt.plot(data._time, data._obs_theta)
    # plt.plot(data._time, data._obs_phi)
    # plt.show()
    # data.plot_action_data()


    exit()

    sf1_data = LoadData("./sim_data/data_sf1.npz")
    sf2_data = LoadData("./sim_data/data_sf2.npz")
    sf3_data = LoadData("./sim_data/data_sf3.npz")
    sf4_data = LoadData("./sim_data/data_sf4.npz")


    #Plotting
    plt.figure()
    plt.plot(sf1_data._time, sf1_data._obs_theta, label="scale factor = 1.0")
    plt.plot(sf2_data._time, sf2_data._obs_theta, label="scale factor = 0.5")
    plt.plot(sf3_data._time, sf3_data._obs_theta, label="scale factor = 0.3")
    plt.ylim(0, 1.0)
    plt.legend(fontsize=15)
    plt.show()

if __name__ == "__main__":
    main()
