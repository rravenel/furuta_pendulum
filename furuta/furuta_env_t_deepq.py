import math
import numpy as np

from gym import spaces
import pybullet as p

import furuta_env_base as feb
import furuta_env_torque as fet
import common as cm
import s2r


s2r.S2R_NETWORK = False

#THROTTLE_PROFILE = [-1.0, -0.5, -0.25, -0.125, 0, 0.125, 0.25, 0.5, 1.0]
#THROTTLE_PROFILE = [-1.0, -0.5, 0, 0.5, 1.0]
THROTTLE_PROFILE = [-1.0, 0, 1.0]
        

class FurutaEnvTorqueDeepq(fet.FurutaEnvTorque):
    def __init__(self, state):
        super(FurutaEnvTorqueDeepq, self).__init__(state=state, action_space=spaces.Discrete(3))
    
    def decodeAction(self, action):
        return THROTTLE_PROFILE[action]
    
    def compute_reward(self, action, done=False):
        return math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.01

class FurutaEnvTorqueDeepqBal(FurutaEnvTorqueDeepq):
    def __init__(self, state):
        super(FurutaEnvTorqueDeepqBal, self).__init__(state)
    
    def reset(self):
        #if self.state == feb.RUN:
        if self.state is None:
            return FurutaEnvTorqueDeepq.reset(
                self, 
                position=0, 
                velocity=0
            )
        
        # position 0rad is upright
        return FurutaEnvTorqueDeepq.reset(
            self, 
            position=cm.sgn()*np.random.uniform(0, cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)), 
            #position=cm.sgn()*cm.deg2Rad(feb.ANGLE_TERMINAL_MIN_D), 
            velocity=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_POLE),
            arm_vel=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_ARM)
        )                   
                                 
    def compute_reward(self, action, done=False):
        # target position
        #arm = math.cos(abs(cm.difRads(self.arm_angle_real, self.arm_angle_target))) * (1 - abs(self.arm_vel_real) / feb.VEL_MAX_ARM)
        arm = math.cos(abs(cm.difRads(self.arm_angle_real, self.arm_angle_target)))
        #arm = abs(cm.difRads(self.arm_angle_real, self.arm_angle_target))
        
        arm_vel = 2 * abs(self.arm_vel_real) / cm.VEL_MAX_ARM
        
        # target velocity
        #arm = (1 - abs(self.arm_vel_real - self.arm_angle_target) / (feb.VEL_MAX_ARM + 6*feb.PI)) * 0.001
        
        pole = math.cos(abs(cm.rad2Norm(self.pole_angle_real)))
        
        #vel = (abs(self.arm_vel_real) / feb.VEL_MAX_ARM) * 2

        throttle = abs(self.decodeAction(action)) #* 0.1
        
        return arm + pole - arm_vel - throttle #- vel
        
    def compute_done(self):
        overspeed = self.overspeed()
        endOfRun = self._envStepCounter >= 1000
        terminalAngle = abs(cm.rad2Norm(self.pole_angle_real)) > cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D)
        return endOfRun or terminalAngle or overspeed


class FurutaEnvTorqueDeepqUp(FurutaEnvTorqueDeepq):
    def __init__(self, state):
        super(FurutaEnvTorqueDeepqUp, self).__init__(state)
    
    def reset(self):
        # position 0rad is upright
        return FurutaEnvTorqueDeepq.reset(
            self, 
            #position=cm.sgn()*np.random.uniform(cm.deg2Rad(feb.ANGLE_TERMINAL_MIN_D), 2*feb.PI), 
            #velocity=cm.sgn()*np.random.uniform(0, feb.VEL_MAX_POLE),
            #arm_vel=cm.sgn()*np.random.uniform(0, feb.VEL_MAX_ARM)

            # start in the spin we never want to see
            position=cm.sgn()*np.random.uniform(cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D), 2*cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D)), 
            #position=cm.sgn()*feb.PI, 
            velocity=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_POLE),
            arm_vel=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_ARM)
        )
        
    def compute_reward(self, action, done=False):
        overspeed = self.overspeed()
        bonus = 0
        if done and not overspeed:
            bonus = 1000 - self._envStepCounter
        return bonus + math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.001 - abs(self.pole_vel_real) / cm.VEL_MAX_POLE
        
    def compute_done(self):
        overspeed = self.overspeed()
        return overspeed or self._envStepCounter >= 1000 or (abs(self.pole_angle_real) < cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D) and abs(self.pole_vel_real) < 2*feb.PI)
        #return self._envStepCounter >= 1000 #or abs(self.pole_angle_jittered) < feb.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)
        