import math
import numpy as np

import pybullet as p

import furuta_env_base as feb
import common as cm
import s2r

# enable hazing
#POS_HAZE = feb.TRAIN
POS_HAZE = None
 
class FurutaEnvPos(feb.FurutaEnvBase):
    def __init__(self, state=None, action_space=0, render=False):
        super(FurutaEnvPos, self).__init__(state=state, action_space=action_space, render=render)
        self.activation_buf = [0, 0] # to penalize abrupt changes via reward
        self.arm_angle_real_prev = 0
        self.arm_target_prev = 0
        
    def reset(self, position=cm.PI, velocity=0, arm_vel=0):
        result = feb.FurutaEnvBase.reset(self, position=position, velocity=velocity, arm_vel=arm_vel)
        self.arm_angle_real_prev = self.arm_angle_real
        self.arm_target_prev = self.arm_angle_real_prev
        return result
    
    def haze(self):
        if abs(self.pole_angle_real) > cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D):
            return
        
        velocity = 0
        if cm.TRAIN == self.state and POS_HAZE == cm.TRAIN:
            if self._envStepCounter % 500 == 0:
                velocity = cm.sgn() * np.random.uniform(0, cm.VEL_MAX_POLE) * 0.3
                
        if cm.RUN == self.state and POS_HAZE == cm.RUN:
            if self._envStepCounter % 500 == 0:
                velocity = cm.sgn() * cm.VEL_MAX_POLE * 0.2
                
        if 0 != velocity:
            p.setJointMotorControl2(bodyUniqueId=self.botId,
                                    jointIndex=self.poleId, 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocity=self.pole_vel_real + velocity)
                                          
    
    def updateSimulation(self, action, botId, jointIndex, overspeed=False): 
        last_err = self.arm_target_prev - self.arm_angle_jittered
        s2r.networkStep(err=last_err)
            
        activation = self.decodeAction(action)
        if overspeed:
            activation = 0.5
        
        self.activation_buf.insert(0, activation)
        self.activation_buf.pop()
        
        target = cm.wrapRad(self.arm_angle_real + activation * cm.MAX_RADIANS)
        self.arm_target_prev = target
        
        err = s2r.networkPredictPos(target)
        target = cm.wrapRad(target + err)
        
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=jointIndex, 
                                controlMode=p.POSITION_CONTROL, 
                                targetPosition=target,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=cm.VEL_MAX_ARM)
                                
        self.arm_angle_real_prev = self.arm_angle_real