import math
import numpy as np

from gym import spaces
import pybullet as p

import furuta_env_base as feb
import furuta_env_pos as fep
import common as cm
import s2r


# Manage s2r
s2r.S2R_DITHER = False
s2r.S2R_RIPPLE = False
s2r.S2R_LATENCY = False
s2r.S2R_NETWORK = False
s2r.S2R_NOISE = False

HISTORY_LEN = 10

class FurutaEnvPosTrpo(fep.FurutaEnvPos):
    def __init__(self, state=None, render=False):
        #super(FurutaEnvPosTrpo, self).__init__(state=state, action_space=spaces.Discrete(3), render=render)
        super(FurutaEnvPosTrpo, self).__init__(state=state, action_space=spaces.Box(np.array([-1]), np.array([1])), render=render)
        self.obs_hist = []
    
    def XsetObs(self):
        obs_min = []
        obs_max = []
        
        min_obs = [-cm.PI, -cm.PI, -cm.PI]
        max_obs = [cm.PI, cm.PI, cm.PI]
        
        while len(obs_min) < HISTORY_LEN * len(min_obs):
            obs_min = obs_min + min_obs
            obs_max = obs_max + max_obs
        
        return obs_min, obs_max
        
    def Xreset(self, position=cm.PI, velocity=0, arm_vel=0):
        result = fep.FurutaEnvPos.reset(self, position=position, velocity=velocity, arm_vel=arm_vel)
        
        obs = [result[0], result[2], result[4]]
         
        while len(self.obs_hist) < HISTORY_LEN * len(obs):
            self.obs_hist = self.obs_hist + obs
        
        return np.array(self.obs_hist)
    
    def Xstep(self, action):
        obs, reward, done, x = fep.FurutaEnvPos.step(self, action)
        
        obs = [obs[0], obs[2], obs[4]]
          
        self.obs_hist = self.obs_hist + obs
        self.obs_hist.pop(0)
        self.obs_hist.pop(0)
        self.obs_hist.pop(0)
        
        return np.array(self.obs_hist), reward, done, {}
    
    def decodeAction(self, action):
        return action
    
    def compute_reward(self, action, done=False):
        return math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.01

class FurutaEnvPosTrpoBal(FurutaEnvPosTrpo):
    def __init__(self, state, render=False):
        super(FurutaEnvPosTrpoBal, self).__init__(state=state, render=render)
    
    def reset(self):        
        # position 0rad is upright
        if False:#self.state == feb.RUN:
            return FurutaEnvPosTrpo.reset(
                self, 
                position=cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D/2), 
                velocity=0,
                arm_vel=0
            )
        
        return FurutaEnvPosTrpo.reset(
            self, 
            position=cm.sgn()*np.random.uniform(0, cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)), 
            velocity=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_POLE),
            arm_vel=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_ARM)
        )                   
    
    # no haze/s2r, 128x128
    # Average reward / Long runs / Average count / Completed / Overspeed (training wall time)
    # 10,000 steps: pole: 4/0/5/0/93 (0.1min)
    # 100,000 steps: pole: 6/0/8/0/44 (0.3min)
    # 1MM steps: pole: 11/7/13/0/81 (2.2min)
    # 2.5MM steps: pole: 13/25/14/0/74 (5.1min)
    # 2.5MM steps: pole + arm: 8/0/8/0/79 (5.6min)
    # 2.5MM steps: pole + 0.1 arm: 15/30/17/0/30 (5.2min)
    # 64x64
    # 2.5MM steps: pole + 0.1 arm: 15/22/16/0/47 (4.5min)
    # 2.5MM steps: pole + 0.01 arm: 11/19/13/0/67
    # 2.5MM steps: pole + 0.1 arm + arm_vel: 6/0/8/0/21
    # 2.5MM steps: pole + 0.1 arm + 0.1 arm_vel: 11/10/12/0/60
    # exchange velocity observations, for history of position observations (len=2)
    # 2.5MM steps: pole: 6/0/8/0/499 (4.9min)
    # 128x128
    # 2.5MM steps: pole: 7/0/9/0/489 (5.3min)
    # history len=3
    # 2.5MM steps: pole: 7/0/8/0/493 (5.3min)
    # history len=10
    # 2.5MM steps: pole: 7/2/9/0/486 (5.4min)
    # no history, include velocities in obs
    # box action space
    # simple reward done or not:
    # 2.5MM steps: 11/11/12/0/5703 (5.8min)
    # 5MM steps: 4/0/5/0/2346
    # 1MM steps: 5/0/6/0/3138 (2.4min)
    # 64x64
    # 1MM steps: 7/0/8/0/3868 (2.1min)
    # 2.5MM steps: pole + 0.1 arm: 4/0/5/0/2582
    # 2.5MM steps: pole: 6/0/8/0/3737
    # oops - disabled excess travel termination, and reverted speed check
    # 2.5MM: pole: 5/0/6/0/25 (5.5min)
    # 128x128
    # 2.5MM: pole: 7/2/8/0/131 (5.6min)
    def compute_reward(self, action, done=False):
        #return 1 if not done else 0
    
    #def compute_reward_x():
        pole = math.cos(abs(cm.rad2Norm(self.pole_angle_real)))

        # target position
        arm = 0#math.cos(abs(cm.difRads(self.arm_angle_real, self.arm_angle_target))) * 0.1
        arm_vel = 0#abs(self.arm_vel_real) / cm.VEL_MAX_ARM * 0.1
        
        activation = 0#abs(self.decodeAction(action)) * 0.0001
        activation_delta = 0#abs(self.activation_buf[0] - self.activation_buf[1])*0.5 * 0.1 # *0.5 normalizes to 1
        
        return pole + arm - arm_vel - activation - activation_delta
        
        
    def compute_done(self):
        overspeed = self.overspeed()
        endOfRun = self._envStepCounter >= 1000
        terminalAngle = abs(cm.rad2Norm(self.pole_angle_real)) > cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D)
        #excessTravel = abs(self.arm_angle_real) > 0.8 * cm.PI # tmp - simple reward
        return endOfRun or terminalAngle or overspeed #or excessTravel


class FurutaEnvPosTrpoUp(FurutaEnvPosTrpo):
    def __init__(self, state, render=False):
        super(FurutaEnvPosTrpoUp, self).__init__(state=state, render=render)
    
    def reset(self):
        # position 0rad is upright
        if self.state == None:#feb.RUN:
            return FurutaEnvPosTrpo.reset(
                self, 
                position=feb.PI, 
                velocity=0,
                arm_vel=0
            )
            
        return FurutaEnvPosTrpo.reset(
            self, 
            position=cm.sgn()*np.random.uniform(cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D), 2*cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D)), 
            velocity=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_POLE),
            arm_vel=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_ARM)
        )
        
        
    def compute_reward(self, action, done=False):
        overspeed = self.overspeed()
        bonus = 0
        if done and not overspeed:
            bonus = 1000 - self._envStepCounter
            #bonus = -self._envStepCounter
        
        pole = math.cos(abs(cm.rad2Norm(self.pole_angle_real)))
        
        pole_vel = 0#(abs(self.pole_vel_real) / cm.VEL_MAX_POLE) * 1
        
        activation = 0#abs(self.decodeAction(action)) * 0.0001
        activation_delta = 0#abs(self.activation_buf[0] - self.activation_buf[1])*0.5 * 0.1 # *0.5 normalizes to 1
            
        return bonus + pole - pole_vel - activation - activation_delta
        
    def compute_done(self):
        overspeed = self.overspeed()
        return overspeed or self._envStepCounter >= 1000 or (abs(self.pole_angle_real) < cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D) and abs(self.pole_vel_real) < 2*feb.PI)
        #return self._envStepCounter >= 1000 #or abs(self.pole_angle_jittered) < feb.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)
        