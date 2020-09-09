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

class FurutaEnvPosPpo(fep.FurutaEnvPos):
    def __init__(self, state=None, render=False):
        super(FurutaEnvPosPpo, self).__init__(state=state, action_space=spaces.Box(np.array([-1]), np.array([1])), render=render)
    
    def decodeAction(self, action):
        return action
    
    # thinking about presenting policy an interface that is not position based, then translating it
    # to position instructions to feed the sim/motor
    
    #def updateSimulation(self, action, botId, jointIndex, overspeed=False): 
    def updateSimulation_x(self, action, botId, jointIndex, overspeed=False): 
        last_err = self.arm_target_prev - self.arm_angle_jittered
        s2r.networkStep(err=last_err)
            
        activation = self.decodeAction(action)
        if overspeed:
            activation = 0.5
        
        self.activation_buf.insert(0, activation)
        self.activation_buf.pop()
        
        target = cm.wrapRad(self.arm_angle_real + activation * cm.MAX_RADIANS)
        self.arm_target_prev = target
        
        err = s2r.networkPredict(self.arm_vel_jittered)
        target = cm.wrapRad(target + err)
        
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=jointIndex, 
                                controlMode=p.POSITION_CONTROL, 
                                targetPosition=target,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=cm.VEL_MAX_ARM)
                                
        self.arm_angle_real_prev = self.arm_angle_real
        
    def compute_reward(self, action, done=False):
        return math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.01

class FurutaEnvPosPpoBal(FurutaEnvPosPpo):
    def __init__(self, state, render=False):
        super(FurutaEnvPosPpoBal, self).__init__(state=state, render=render)
    
    def reset(self):        
        # position 0rad is upright
        if False:#self.state == feb.RUN:
            return FurutaEnvPosPpo.reset(
                self, 
                position=cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D/2), 
                #position=cm.deg2Rad(1), 
                velocity=0,
                arm_vel=0
            )
        
        return FurutaEnvPosPpo.reset(
            self, 
            position=cm.sgn()*np.random.uniform(0, cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)), 
            velocity=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_POLE),
            arm_vel=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_ARM)
        )                   
                        
    # baseline, cos pole only: 4 long runs
    # add full arm position: 3 long
    # run for 1MM steps - about the same
    # pole only 5MM: 39/51 (average reward, long runs)
    # add full arm: 21/42
    # 0.1 arm: 36:54
    # 0.01 arm: 24/33
    # 0.1 arm, 
    ###########
    # no network: Average reward / Long runs / Average count / Completed / Overspeed
    # 5MM: pole: 32/-9/33/0
    # 2.5MM: pole: 25/8/27/0
    # 2.5MM: pole + arm: 30/0/23/0
    # 2.5MM: pole + 0.1 arm: 23/-31/24/0
    # 2.5MM: pole + 0.1 arm: 31/56/33/0/56
    # 2.5MM: pole + 0.01 arm: 22/35/23/0/66
    # 2.5MM: pole + 0.1 arm + arm_vel: 7/0/8/0/6
    # 2.5MM: pole + 0.1 arm + 0.1 arm_vel: 22/40/24/0/37
    # 2.5MM: pole + 0.1 arm + 0.01 arm_vel: 20/39/22/0/43
    # 2.5MM: pole + 0.1 arm + 0.001 arm_vel: 25/47/26/0/50
    # 2.5MM: pole + 0.1 arm + 0.0001 arm_vel: 22/36/23/0/11
    # 5MM: pole + 0.1 arm + 0.0001 arm_vel: 33/51/35/0/46
    # disable all s2r
    # 2.5MM: pole: 18/29/20/0/50
    # 2.5MM: pole + 0.1 arm + 0.0001 arm_vel: 27/41/29/0/58
    # 2.5MM: pole + 0.1 arm + 0.00001 arm_vel: 19/31/21/0/36
    # tf on all cores
    # 2.5MM: pole + 0.1 arm + 0.00001 arm_vel: 33/54/34/0/46
    # 64x64
    # 2.5MM: pole + 0.1 arm + 0.00001 arm_vel: 16/31/18/0/51 (4.8min)
    # 128x128
    # 10MM: pole + 0.1 arm + 0.00001 arm_vel: 34/39/36/0/54 (20min)
    # box space -1:1
    # 2.5MM: pole + 0.1 arm + 0.00001 arm_vel: 59/31/59/24/147 (5.4min)
    # 2.5MM: not done: 61/28/62/28/47
    # 2.5MM: not done - no travel limit: 58/27/59/27/153 (5.3min) 
    # 5MM: not done - no travel limit: 65/31/66/30/153 (~10min)
    # 2.5MM: not done: 
    def compute_reward(self, action, done=False):
        return 1 if not done else 0
    
    def compute_reward_x(self, action, done=False):
        pole = math.cos(abs(cm.rad2Norm(self.pole_angle_real)))

        # target position
        arm = 0#math.cos(abs(cm.difRads(self.arm_angle_real, self.arm_angle_target))) * 0.1
        arm_vel = 0#abs(self.arm_vel_real) / cm.VEL_MAX_ARM * 0.00001
        
        activation = 0#abs(self.decodeAction(action)) * 0.0001
        activation_delta = 0#abs(self.activation_buf[0] - self.activation_buf[1])*0.5 * 0.1 # *0.5 normalizes to 1
        
        return pole + arm - arm_vel - activation - activation_delta
        
        
    def compute_done(self):
        overspeed = self.overspeed()
        endOfRun = self._envStepCounter >= 1000
        terminalAngle = abs(cm.rad2Norm(self.pole_angle_real)) > cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D)
        excessTravel = abs(self.arm_angle_real) > 0.8 * cm.PI # tmp - simple reward
        return endOfRun or terminalAngle or overspeed or excessTravel


class FurutaEnvPosPpoUp(FurutaEnvPosPpo):
    def __init__(self, state, render=False):
        super(FurutaEnvPosPpoUp, self).__init__(state=state, render=render)
    
    def reset(self):
        # position 0rad is upright
        if self.state == None:#feb.RUN:
            return FurutaEnvPosPpo.reset(
                self, 
                position=feb.PI, 
                velocity=0,
                arm_vel=0
            )
            
        return FurutaEnvPosPpo.reset(
            self, 
            position=cm.sgn()*np.random.uniform(cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D), 2*cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D)), 
            velocity=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_POLE),
            arm_vel=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_ARM)
        )
        
    # pole + 0.1 pole_vel = 176/8 (average reward / short runs out of 100)
    # pole + 1 pole_vel = 113/5 (average reward / short runs out of 100)
    # same but 1MM -> 5MM: 910/60!
    # added s2r: dither, ripple, latency: 560/10 :(
    # full activation delta: 885/49
    # 0.1 delta: 885/54
    # box space -1:1, 128x128, bonus reward: 928/32/71 (4.6min)
    def compute_reward(self, action, done=False):        
        overspeed = self.overspeed()
        if done and not overspeed:
            return 1000 - self._envStepCounter
        return 0
    
    def compute_reward_x():
        overspeed = self.overspeed()
        bonus = 0
        if done and not overspeed:
            bonus = 1000 - self._envStepCounter
        
        pole = math.cos(abs(cm.rad2Norm(self.pole_angle_real)))
        
        pole_vel = (abs(self.pole_vel_real) / cm.VEL_MAX_POLE) * 1
        
        activation = 0#abs(self.decodeAction(action)) * 0.0001
        activation_delta = abs(self.activation_buf[0] - self.activation_buf[1])*0.5 * 0.1 # *0.5 normalizes to 1
            
        return bonus + pole - pole_vel - activation - activation_delta
        
    def compute_done(self):
        overspeed = self.overspeed()
        endOfRun = self._envStepCounter >= 1000
        terminalAngle = abs(cm.rad2Norm(self.pole_angle_real)) < cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)
        #excessTravel = abs(self.arm_angle_real) > 0.8 * cm.PI # tmp - simple reward
        return endOfRun or terminalAngle or overspeed #or excessTravel
        #return overspeed or self._envStepCounter >= 1000 or (abs(self.pole_angle_real) < cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D) and abs(self.pole_vel_real) < 2*feb.PI)
        #return self._envStepCounter >= 1000 #or abs(self.pole_angle_jittered) < feb.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)
        