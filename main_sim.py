import os
import gym
import numpy as np
import rock_walk
import time
import pybullet as bullet
import matplotlib.pyplot as plt

from stable_baselines3 import TD3, SAC, PPO
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecVideoRecorder


class RLModel:

    def __init__(self, connection, freq, frame_skip, train):
        self._env = gym.make("RockWalk-v0", bullet_connection=connection, step_freq=freq, frame_skip=frame_skip)

        if train == True:
            self._env = Monitor(self._env, "./log")
        else:
            pass

    def train_model(self):
        n_actions = self._env.action_space.shape[-1]
        action_noise = OrnsteinUhlenbeckActionNoise(mean=0*np.ones(n_actions), sigma=1.5*np.ones(n_actions)) #5 prev

        self._model = SAC("MlpPolicy", self._env,
                          action_noise=action_noise,
                          batch_size=128,
                          train_freq= 64,
                          gradient_steps= 64,
                          learning_starts=20000,
                          verbose=1,
                          tensorboard_log = "./rockwalk_tb/")

        checkpoint_callback = CheckpointCallback(save_freq=50000, save_path='./save/', name_prefix='rw_model')
        self._model.learn(total_timesteps=500000, log_interval=10, callback=checkpoint_callback)
        self._model.save_replay_buffer('./save/buffer')
        self._env.close()


    def test_model(self, freq):
        self._trained_model = SAC.load("./save/rw_model_150000_steps", device="cpu")
        print("Trained model loaded")
        obs = self._env.reset()

        obs_psi_list = list()
        obs_theta_list = list()
        obs_phi_list = list()
        obs_psi_dot_list = list()
        obs_theta_dot_list = list()
        obs_phi_dot_list = list()

        action_x_list = list()
        action_y_list = list()

        action_time_list = list()

        for count in range(5000):
            action, _states = self._trained_model.predict(obs, deterministic=True)
            obs, rewards, dones, info = self._env.step(action)

            true_cone_state = self._env.cone.get_observation()
            # print(true_cone_state[2]+np.pi/2)
            # print(action)

            obs_psi_list.append(true_cone_state[2])
            obs_theta_list.append(true_cone_state[3])
            obs_phi_list.append(true_cone_state[4])
            obs_psi_dot_list.append(true_cone_state[7])
            obs_theta_dot_list.append(true_cone_state[8])
            obs_phi_dot_list.append(true_cone_state[9])

            action_x_list.append(action[0])
            action_y_list.append(action[1])

            action_time_list.append(time.time())

            time.sleep(1./240.)

        obs_psi_np = np.array(obs_psi_list)
        obs_theta_np = np.array(obs_theta_list)
        obs_phi_np = np.array(obs_phi_list)
        obs_psi_dot_np = np.array(obs_psi_dot_list)
        obs_theta_dot_np = np.array(obs_theta_dot_list)
        obs_phi_dot_np = np.array(obs_phi_dot_list)

        action_x_np = np.array(action_x_list)
        action_y_np = np.array(action_y_list)
        action_time_np = np.array(action_time_list)

        np.savez("./sim_data/data.npz",
                 obs_psi = obs_psi_np,
                 obs_theta = obs_theta_np,
                 obs_phi = obs_phi_np,
                 obs_psi_dot = obs_psi_dot_np,
                 obs_theta_dot = obs_theta_dot_np,
                 obs_phi_dot = obs_phi_dot_np,
                 action_x=action_x_np,
                 action_y=action_y_np,
                 time=action_time_np)


def main():
    freq = 50.
    frame_skip = 5
    # rl_model = RLModel(0, freq, frame_skip, train=True)
    # rl_model.train_model()

    test_begin = input("Press enter to test model")
    if test_begin == "":
        freq = 240.
        frame_skip = 1
        rl_model = RLModel(1, freq, frame_skip, train=False) #0: DIRECT 1: GUI 2: SHARED_MEMORY
        rl_model.test_model(freq)
    else:
        rl_model._env.close()


if __name__ == "__main__":
    main()



# self._model = TD3("MlpPolicy", self._env,
#                   action_noise=action_noise,
#                   batch_size=128,
#                   verbose=1,
#                   tensorboard_log = "./rockwalk_tb/")




        # st_time = time.time()
        # for i in range(100000):
        #     curr_time = time.time()-st_time
        #
        #     action = np.array([0, 0.5*np.sin(np.pi*curr_time)])
        #     self._env.eef.apply_action(action)
        #     bullet.stepSimulation()
        #
        #     time.sleep(1/240.)



        # for i in range(100000):
        #     bullet.stepSimulation()
        #     time.sleep(1/240.)
        #     cone_state = self._env.cone.get_observation()
        #     print(cone_state[0])






# self._model = TD3.load("good_td3_trained_rockwalk_model", env = self._env)
# action_noise = NormalActionNoise(mean=0*np.ones(n_actions), sigma=1.5*np.ones(n_actions))
# self._env.save("./save/vec_normalize.pkl")

# bullet.stopStateLogging(logID)
# logID = bullet.startStateLogging(bullet.STATE_LOGGING_VIDEO_MP4, "test_video")
