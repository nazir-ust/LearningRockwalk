from stable_baselines3.common.callbacks import BaseCallback


class GenerateObjectCallback(BaseCallback):
    """
    A custom callback that derives from ``BaseCallback``.

    :param verbose: (int) Verbosity level 0: not output 1: info 2: debug
    """
    def __init__(self, check_freq: int, verbose=1):
        super(GenerateObjectCallback, self).__init__(verbose)
        self.check_freq = check_freq

    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        pass


    def _on_rollout_start(self) -> None:
        """
        A rollout is the collection of environment interaction
        using the current policy.
        This event is triggered before collecting new samples.
        """
        pass

    def _on_step(self) -> bool:
        """
        This method will be called by the model after each call to `env.step()`.
        :return: (bool) If the callback returns False, training is aborted early.
        """
        if self.n_calls % self.check_freq == 0:
            env = self.training_env.envs[0]
            print("GENERATING NEW OBJECT")

            ellipse_a = env.np_random.uniform(0.25,0.5) #length of semi-major/minor along x-axis
            ellipse_b = env.np_random.uniform(0.25,0.5) #length of semi-major/minor along y-axis

            apex_x = 0. #env.np_random.uniform(-ellipse_a,ellipse_a)
            apex_y = env.np_random.uniform(-ellipse_b,0)
            apex_z = env.np_random.uniform(1.0,1.5)

            with open('training_objects_params.txt', 'a') as f:
                f.write(str(ellipse_a)+","+str(ellipse_b)+","+str(apex_x)+","+str(apex_y)+","+str(apex_z)+"\n")

            env.cone.generate_object_mesh(ellipse_params=[ellipse_a, ellipse_b],
                                          apex_coordinates=[apex_x, apex_y, apex_z], density=10)
            env.cone.generate_urdf_file()

        return True

    # def generate_random_object(group):
    #     if group == 1:
    #         ""
    #
    #     elif group == 2:
    #
    #     elif group == 3:

    def _on_rollout_end(self) -> None:
        """
        This event is triggered before updating the policy.
        """
        pass

    def _on_training_end(self) -> None:
        """
        This event is triggered before exiting the `learn()` method.
        """
        pass
