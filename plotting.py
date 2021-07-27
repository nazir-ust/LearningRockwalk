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



    def plot_action_data(self):
        plt.figure()
        plt.plot(self._time, self._action_x)
        plt.plot(self._time, self._action_y)
        plt.show()


def main():

    data = LoadData("./test_data/data.txt")

    # Create two subplots and unpack the output array immediately
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, sharex=True)
    fig.set_size_inches(18.5, 10.5)
    ax1.plot(data._time, data._obs_x, label='x_CM')
    ax1.plot(data._time, data._obs_y, label='y_CM')
    ax1.legend()
    ax1.set_title('Position of cone center of mass in meters')

    ax2.plot(data._time, data._obs_theta, label='nutation')
    ax2.plot(data._time, np.radians(25)*np.ones([np.size(data._time)]), label='desired nutation')
    ax2.set_ylim(0, np.pi/2)
    ax2.legend()
    ax2.set_title('Nutation angle in radians')

    ax3.plot(data._time, data._obs_phi)
    ax3.set_ylim(-np.pi, np.pi)
    ax3.set_title('Spin angle in radians')

    ax4.plot(data._time, data._cone_energy, label='Energy of CoM')
    ax4.set_ylim(3.5,5.5)
    ax4.set_title('Cone energy')

    ax5.plot(data._time, data._action_x, label='Vx_C')
    ax5.plot(data._time, data._action_y, label='Vy_C')
    ax5.legend()
    ax5.set_title('Actions returned by agent: Velocity of control point long x- and y-axes in meters per second **Actual magnitude is scaled**')
    ax5.set_xlabel('Time (s)')
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
