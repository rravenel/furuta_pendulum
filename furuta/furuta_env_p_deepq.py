import math
import numpy as np

from gym import spaces
import pybullet as p

import furuta_env_base as feb
import furuta_env_pos as fep
import common as cm
import s2r


# Manage s2r
s2r.S2R_DITHER = True
s2r.S2R_RIPPLE = True
s2r.S2R_LATENCY = True
s2r.S2R_NETWORK = True
s2r.S2R_NOISE = False

s2r.NETWORK_PATH = "tf/rnd_obs_policy_network_3x32-100e.h5" #"tf/rnd_obs_policy_network_3x32.h5"
s2r.OBSERVATION_HISTORY = 2


POSITION_PROFILE = [-1.0, -0.5, -0.25, -0.125, 0, 0.125, 0.25, 0.5, 1.0]
        

class FurutaEnvPosDeepq(fep.FurutaEnvPos):
    def __init__(self, state=None, render=False):
        super(FurutaEnvPosDeepq, self).__init__(state=state, action_space=spaces.Discrete(9), render=render)
    
    def decodeAction(self, action):
        return POSITION_PROFILE[action]
    
    def compute_reward(self, action, done=False):
        return math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.01

class FurutaEnvPosDeepqBal(FurutaEnvPosDeepq):
    def __init__(self, state, render=False):
        super(FurutaEnvPosDeepqBal, self).__init__(state=state, render=render)
    
    def reset(self):
        arm_vel = 0
        val = None
        #if self.state == feb.RUN:
        if self.state is None:
            val =  FurutaEnvPosDeepq.reset(
                self, 
                position=0, 
                velocity=0
            )
        else:
            arm_vel=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_ARM)
            # position 0rad is upright
            val =  FurutaEnvPosDeepq.reset(
                self, 
                position=cm.sgn()*np.random.uniform(0, cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D)), 
                #position=cm.sgn()*cm.deg2Rad(feb.ANGLE_TERMINAL_MIN_D), 
                velocity=cm.sgn()*np.random.uniform(0, cm.VEL_MAX_POLE),
                #velocity=cm.sgn()*np.random.uniform(feb.VEL_MAX_POLE/2, feb.VEL_MAX_POLE),
                arm_vel=arm_vel
                #arm_vel=cm.sgn()*np.random.uniform(feb.VEL_MAX_ARM/2, feb.VEL_MAX_ARM)
            )
        
        self.arm_angle_real_prev = self.arm_angle_real - arm_vel * cm.MAX_RADIANS
        return val                           

    # LAST: delta: 0.01 -> 0.001; arm 0.001 - fail, lots of standing falls, arm motion too constraing
    # delta 0.001 -> 0.0001 - fails, but moves more
    # delta 0.0001 -> 0.00001 - still fails
    # nothing but pole - mostly falls, occasional spin
    # turn on arm at 0.01 - same, but a bit more spin
    # arm 0.01 -> 0.1 - same
    # arm off, delta on at 0.01 - fail, infrequent spin
    # delta 0.01 -> 1 - a few good balances
    # raised min pole vel on reset to 1/2 max
    
    # pseudo accelleration control (average reward / long runs (>100) / average count / 1000 count)
    # 100,000 steps: pole only baseline: 2/0/?
    # 100,000 steps: initialize arm vel per reset(): 4/0/5
    # 100,000 steps: pole + arm: 8/9/11
    # 100,000 steps: pole + 0.1 arm: 26/16/27 (a few completes?)
    # 100,000 steps: pole + 0.01 arm: 14/9/16 (a few completes?)
    # 100,000 steps: pole + 0.1 arm + arm_vel: 5/15/9/0
    # 100,000 steps: pole + 0.1 arm + 0.1 arm_vel: 7/11/8/0
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel: 33/17/34/13, 35/20/37/15
    # 100,000 steps: pole + 0.1 arm + 0.0001 arm_vel: 7/3/8/0
    # 100,000 steps: pole + 0.01 arm + 0.001 arm_vel: 6/5/7/0
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel + activation: 17/8/21/8
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel + 0.1 activation: 9/5/10/1
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel + 0.001 activation: 24/15/35/15
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel + 0.0001 activation: 7/6/8/0
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel + 0.001 activation + delta: 8/9/10/1, 7/5/10/1
    # 200,000 steps: pole + 0.1 arm + 0.001 arm_vel: 24/13/26/10
    # normal position control
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel: 21/20/22/3
    # 100,000 steps: pole + 0.01 arm + 0.001 arm_vel: 26/29/27/4
    # 100,000 steps: pole + arm + 0.001 arm_vel: 31/17/31/11
    # 100,000 steps: pole + 0.1 arm + 0.0001 arm_vel: 47/31/48/19
    # 100,000 steps: pole + arm + 0.0001 arm_vel: 77/38/76/34 -> hybrid test/100: 826/97
    # enable hazing
    # 100,000 steps: pole + 0.1 arm + 0.0001 arm_vel: 9/4/10/0
    # 100,000 steps: pole + arm + 0.0001 arm_vel: 20/14/17/2 -> 558/97
    # disable hazing
    # 100,000 steps: pole + arm + 0.0001 arm_vel: 43/18/28/8, 54/21/38/15 -> hybrid/50: 830/49
    # 200,000 steps: pole + arm + 0.0001 arm_vel: 105/30/62/27 -> hybrid/50: 932
    # add motor network 3x32
    # 200,000 steps: pole + arm + 0.0001 arm_vel: 10/5/14/0
    # 100,000 steps: pole + arm + 0.0001 arm_vel + activation_delta: 58/23/50/21
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.1 activation_delta: 79/26/55/24 <--
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.01 activation_delta: 12/5/11/1
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.1 activation_delta + activation: 18/8/17/3
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.01 activation_delta + activation: 68/28/54/22
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.1 activation_delta + 0.1 activation: 27/20/22/4
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.1 activation_delta + 0.01 activation: 7/0/8/0
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.01 activation_delta + 0.1 activation: 13/7/14/0
    # updated data set for intermediate network, softsign, MSE loss function
    # 100,000 steps: pole + arm + 0.0001 arm_vel: 28/-33/24/4
    # 100,000 steps: pole + arm + 0.0001 arm_vel + 0.1 activation_delta: 22/-38-16/3
    # 100,000 steps: pole only baseline: 28/-12/29/9
    # add latency
    # 100,000 steps: pole: 12/-108/14/1
    # add noise
    # 100,000 steps: pole: 8/-33/10/0, 9/-141/10/0
    # 100,000 steps: pole + arm: 12/-80/15/0
    # 100,000 steps: pole + arm + 0.0001 arm_vel: 9/-96/9/0
    # 200,000 steps: pole + arm + 0.0001 arm_vel: 12/-198/10/0
    # no noise
    # 100,000 steps: pole + arm + 0.0001 arm_vel: 27/-88/18/2
    # 100,000 steps: pole + 0.1 arm: 43/-62/46/13 <--
    # 100,000 steps: pole + arm: 12/-36/16/2
    # 100,000 steps: pole + 0.01 arm: 8/-90/10/0
    # 100,000 steps: pole + 0.1 arm + 0.0001 arm_vel: 33/-94/33/12
    # 100,000 steps: pole + 0.1 arm + 0.001 arm_vel: 8/-128/9/0
    # 100,000 steps: pole + 0.1 arm + 0.00001 arm_vel: 9/-50/10/0
    # 100,000 steps: pole + 0.1 arm + activation: 52/-77/68/31, 21/-106/25/7, 20/-122/24/6 <--
    # 100,000 steps: pole + 0.1 arm + 0.01 activation: 28/-62/31/11
    # 100,000 steps: pole + 0.1 arm + 0.1 activation: 17/-51/19/0
    # 100,000 steps: pole + 0.1 arm + activation + activation_delta: 25/-109/33/11
    # 100,000 steps: pole + 0.1 arm + activation + 0.01 activation_delta: 52/-80/62/28, 9/-130/13/2, 41/-104/51/20, 61/-91/68/31, 58/-92/72/33 <-- 372/-714/0
    # 100,000 steps: pole + 0.1 arm + activation + 0.0001 activation_delta: 28/-92/36/13
    def compute_reward(self, action, done=False):
        pole = math.cos(abs(cm.rad2Norm(self.pole_angle_real)))
        
        # target position
        arm = math.cos(abs(cm.difRads(self.arm_angle_real, self.arm_angle_target))) * 0.1
        #arm = abs(cm.difRads(self.arm_angle_real, self.arm_angle_target))
        arm_vel = 0#(abs(self.arm_vel_real) / cm.VEL_MAX_ARM) * 0.00001
                
        activation = abs(self.decodeAction(action)) #* 0.1
        activation_delta = (abs(self.activation_buf[0] - self.activation_buf[1])/2) * 0.01 # max value is 2 so scale to 1
                
        return pole + arm - arm_vel - activation - activation_delta
        
    def compute_done(self):
        overspeed = self.overspeed()
        endOfRun = self._envStepCounter >= 1000
        terminalAngle = abs(cm.rad2Norm(self.pole_angle_real)) > cm.deg2Rad(cm.ANGLE_TERMINAL_MAX_D)
        return endOfRun or terminalAngle or overspeed


class FurutaEnvPosDeepqUp(FurutaEnvPosDeepq):
    def __init__(self, state, render=False):
        super(FurutaEnvPosDeepqUp, self).__init__(state=state, render=render)
    
    def reset(self):
        # position 0rad is upright
        return FurutaEnvPosDeepq.reset(
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
    
    # ***Average reward: 539.668	Average count: 51.350	Short runs: 70
    # 100,000 steps: bonus + pole + 0.1 pole_vel: 540/51/70 
    # 100,000 steps: bonus + pole: 647/48/63
    # 100,000 steps: bonus + pole + pole_vel: 645/47/73
    # 100,000 steps: bonus + pole + pole_vel + activation: 645/46/61
    # 100,000 steps: bonus + pole + pole_vel + 0.1 activation: 666/38/77
    # updated data set for intermediate network, softsign, MSE loss function
    # add network, latency
    # 100,000 steps: bonus + pole: 611/34/50
    # 100,000 steps: bonus + pole + pole_vel: 679/35/33
    # 100,000 steps: bonus + pole + pole_vel + 0.1 activation: 754/50/31
    def compute_reward(self, action, done=False):
        overspeed = self.overspeed()
        bonus = 0
        if done and not overspeed:
            bonus = 1000 - self._envStepCounter
        
        pole = math.cos(abs(cm.rad2Norm(self.pole_angle_real)))
        
        pole_vel = (abs(self.pole_vel_real) / cm.VEL_MAX_POLE) #* 0.1
        
        activation = abs(self.decodeAction(action)) * 0.1
        activation_delta = 0#abs(self.activation_buf[0] - self.activation_buf[1]) * 0.1
            
        return bonus + pole - pole_vel - activation - activation_delta
        
    def compute_done(self):
        overspeed = self.overspeed()
        return overspeed or self._envStepCounter >= 1000 or (abs(self.pole_angle_real) < cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D) and abs(self.pole_vel_real) < 2*feb.PI)
        #return self._envStepCounter >= 1000 #or abs(self.pole_angle_jittered) < feb.deg2Rad(feb.ANGLE_TERMINAL_MIN_D)



class FurutaEnvPosDeepqTest(FurutaEnvPosDeepq):
    def __init__(self, state, render=False):
        super(FurutaEnvPosDeepqTest, self).__init__(state=state, render=render)
        self.arm_angle_real_prev = None
    
    def reset(self, position=cm.PI, velocity=0, arm_vel=0):
        result =  FurutaEnvPosDeepq.reset(self, position=position, velocity=velocity, arm_vel=arm_vel)
        self.arm_angle_real_prev = result[0]
        return result
        
    # steady speed    
    def compute_reward(self, action, done=False):
        #speed = 1 # Hz
        #rads = speed * 2 * cm.PI
        #delta = rads * cm.STEP
        
        #target = cm.wrapRad(cm.norm2Rad(self.arm_angle_real_prev) + delta)
        
        target = cm.wrapRad(2 * cm.PI * self._envStepCounter / 100)
        dif = cm.difRads(cm.norm2Rad(self.arm_angle_real), target)
        
        #self.arm_angle_real_prev = self.arm_angle_real
        
        reward = math.cos(abs(dif))

        print("%d:\tTarg: %.3f\tStep: %.3f\tPos: %.3f\tErr: %.3f\tRew: %.3f" % (self._envStepCounter, 
                                                                            cm.rad2Deg(target), 
                                                                            cm.deg2Norm(cm.rad2Deg(self.decodeAction(action) * cm.MAX_RADIANS)), 
                                                                            cm.rad2Deg(cm.norm2Rad(self.arm_angle_real)), 
                                                                            cm.deg2Norm(cm.rad2Deg(dif)), 
                                                                            reward))

        return reward

    def compute_done(self):
        overspeed = False#self.overspeed()
        return overspeed or self._envStepCounter >= 1000 
        