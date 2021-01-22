import math
import numpy as np

from gym import spaces

import furuta_env_torque as fet
import common as cm


class FurutaEnvTorquePpo2(fet.FurutaEnvTorque):
    def __init__(self, state, render=False):
        super(FurutaEnvTorquePpo2, self).__init__(state=state, action_space=spaces.Box(np.array([-1]), np.array([1])), render=render)
    
    def decodeAction(self, action):
        return action
    
    def compute_reward(self, action, done=None):
        return math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.0001

