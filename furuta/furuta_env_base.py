import os
import math
import numpy as np
import time

import gym
from gym import spaces
import pybullet as p
import pybullet_data

import common as cm
import s2r

'''
This version relies on the furuta-v3 conda environment.

Base FurutaEnv

Implementations MUST define:
    self.action_space
        Defined type is dependent on requirements of implemented RL algorithm

    assign_throttle(self, action)
        Defined type is dependent on requirements of implemented RL algorithm

Implementations MAY define:
    compute_reward(self)
        Overriding this enables differentiation unique to implemented RL algorithm, as well as 
        different strategies, such as whether to separate the spin-up task from balancing

'''
PI = cm.PI # convenience

DITHER = 0.15 # +/- 15%
DAMPING = 0.0008

# Manage s2r
s2r.S2R_DITHER = True
s2r.S2R_RIPPLE = True
s2r.S2R_LATENCY = True
s2r.S2R_NETWORK = True
s2r.S2R_NOISE = False


# control by setting motor current
class FurutaEnvBase(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self, render=False, state=None, action_space=None):
        super(FurutaEnvBase, self).__init__()
        
        if None is state:
            print("state is None. Exiting.")
            exit()   
        self.state = state
              
        if None is action_space:
            print("action_space is None. Exiting.")
            exit()   
        self.action_space = action_space
        
        obs_min, obs_max = self.setObs()
        
        self.observation_space = spaces.Box(np.array(obs_min), np.array(obs_max))
        
        s2r.networkInit()
        
        self.num_envs = 2
        
        s2r.init()
        self.setRender(render)

    def setObs(self):
        obs_min = [-PI, -cm.VEL_MAX_ARM, -PI, -cm.VEL_MAX_POLE, -PI]
        obs_max = [PI, cm.VEL_MAX_ARM, PI, cm.VEL_MAX_POLE, PI]
        
        return obs_min, obs_max
    
    # must be called immediately following __init__()
    def setRender(self, render):
        if (render):
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        
    def reset(self, position=PI, velocity=0, arm_vel=0):
        self.throttle = 0
        self.action = None
        self._envStepCounter = 0

        self.arm_angle_target = np.random.uniform(0, 2*PI)
        #self.arm_vel_target = np.random.uniform(-6*PI, 6*PI) # velocity rad/s, 3Hz
        
        self.arm_angle_real = 0
        self.arm_vel_real = 0

        self.arm_angle_jittered = 0
        self.arm_vel_jittered = 0

        self.pole_angle_real = 0
        self.pole_vel_real = 0
        
        p.resetSimulation()
        p.setGravity(0,0,-9.8) # m/s^2
        p.setTimeStep(cm.STEP) # sec
        planeId = p.loadURDF("plane.urdf")
        
        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "furuta.urdf"), 
            useFixedBase=1, 
            flags=p.URDF_USE_INERTIA_FROM_FILE)
                
        numJoints = p.getNumJoints(self.botId)
        joints = {}
        for j in range(numJoints):
            joint = p.getJointInfo(self.botId, j)
            joints[joint[1]] = j           
        self.armId = joints[b'pillar_to_arm']
        self.poleId = joints[b'axle_to_rod']
        
        s2r.ditherInertia(self.botId, DITHER, DAMPING)

        p.resetJointState(bodyUniqueId=self.botId, 
                                jointIndex=self.poleId, 
                                targetValue=position,
                                targetVelocity=velocity)

        p.resetJointState(bodyUniqueId=self.botId, 
                                jointIndex=self.armId, 
                                targetValue=0,
                                targetVelocity=arm_vel)
                                
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=self.armId, 
                                controlMode=p.VELOCITY_CONTROL, 
                                force=0)

        obs = self.compute_observation()
        obs[0] = cm.rad2Norm(obs[0])
        obs[2] = cm.rad2Norm(obs[2])
        
        s2r.networkReset(obs)
                    
        return np.array(obs)

    def overspeed(self):
        return abs(self.arm_vel_real) > cm.VEL_MAX_ARM or abs(self.pole_vel_real) > cm.VEL_MAX_POLE
        
    def step(self, action):
        self.action = action
        
        last_arm_angle = self.arm_angle_real
        last_pole_angle = self.pole_angle_real
        
        self.updateSimulation(action, self.botId, self.armId, overspeed=self.overspeed())

        self.haze()

        p.stepSimulation()
        
        self.compute_observation()

        latency = s2r.latency(self.arm_angle_real, last_arm_angle, self.pole_angle_real, last_pole_angle, cm.STEP)
        arm_angle_jittered = latency[0]
        arm_vel_jittered = latency[1]
        pole_angle_jittered = latency[2]
        pole_vel_jittered = latency[3]

        self.arm_angle_jittered = arm_angle_jittered
        self.arm_vel_hittered = arm_vel_jittered
        
        obs = [arm_angle_jittered, arm_vel_jittered, pole_angle_jittered, pole_vel_jittered, self.arm_angle_target]
        
        s2r.networkStep(obs=obs)
                
        done = self.compute_done()
        reward = self.compute_reward(action, done=done)
        
        self._envStepCounter += 1

        return np.array(obs), reward, done, {}
    
    def decodeAction(self, action):
        raise NotImplementedError
    
    def updateSimulation(self, action): 
        raise NotImplementedError

    # must be implemented by inheriting class based on control paradigm (ie torque vs position)
    def haze(self):
        return

    def compute_observation(self):
        arm_angle_real, self.arm_vel_real, _, _ = p.getJointState(self.botId, self.armId)
        self.arm_angle_real = cm.wrapRad(arm_angle_real)
                
        arm_to_axle_id = self.poleId
        pole_angle_real, self.pole_vel_real, _, _ = p.getJointState(self.botId, self.poleId)
        self.pole_angle_real = cm.wrapRad(pole_angle_real)

        self.arm_angle_real = s2r.noise(self.arm_angle_real, cm.NOISE_SENSE)
        self.arm_vel_real = s2r.noise(self.arm_vel_real, cm.NOISE_SENSE)
        self.pole_angle_real = s2r.noise(self.pole_angle_real, cm.NOISE_SENSE)
        self.pole_vel_real = s2r.noise(self.pole_vel_real, cm.NOISE_SENSE)

        obs = [self.arm_angle_real, self.arm_vel_real, self.pole_angle_real, self.pole_vel_real, self.arm_angle_target]
        return obs
        
    def compute_reward(self, action, done=False):
        # Implementations may override
        return math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.0001    

    def compute_done(self):
        return self._envStepCounter >= 1000 # 10s
        
    def render(self, mode='human', close=False):
        pass

