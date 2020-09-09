import math
import numpy as np

import pybullet as p

import furuta_env_base as feb
import common as cm
import s2r


class FurutaEnvTorque(feb.FurutaEnvBase):
    def __init__(self, state, action_space):
        super(FurutaEnvTorque, self).__init__(state=state, action_space=action_space)
    
    def haze(self):
        if abs(self.pole_angle_real) > cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D):
            return
        
        velocity = 0
        if cm.TRAIN == self.state:
            return
            if self._envStepCounter % 500 == 0:
                velocity = cm.sgn() * np.random.uniform(0, cm.VEL_MAX_POLE) * 0.3
                
        if cm.RUN == self.state:
            return
            if self._envStepCounter % 500 == 0:
                velocity = cm.sgn() * cm.VEL_MAX_POLE * 0.2
                
        if 0 != velocity:
            p.setJointMotorControl2(bodyUniqueId=self.botId,
                                    jointIndex=self.poleId, 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocity=self.pole_vel_real + velocity)
                                    
    def updateSimulation(self, action, botId, jointIndex, overspeed=False): 
        throttle = 0
        if not overspeed:
            throttle = self.decodeAction(action)
        
        current = throttle * cm.MAX_CURRENT
        current = s2r.networkPredict(current)
        
        torque = current * cm.TORQUE_CONSTANT
        torque += s2r.rad2rip(self.arm_angle_real)
        torque = s2r.noise(torque, cm.NOISE_COMMAND)
        
        p.setJointMotorControl2(bodyUniqueId=botId,
                                jointIndex=jointIndex, 
                                controlMode=p.TORQUE_CONTROL, 
                                force=torque)
    