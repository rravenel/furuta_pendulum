import numpy as np
import time

import pybullet as p
import tensorflow as tf

import common as cm
import comm_delay as cd

'''
Sim-to-Real Library

'''

# Set by calling code
S2R_DITHER = None
S2R_RIPPLE = None
S2R_LATENCY = None
S2R_NETWORK = None
S2R_NOISE = None

def init():
    if S2R_RIPPLE:
        global rip
        rip = ripple_map()

# Add random communication error (aka noise)

def noise(val, err):
    if not S2R_NOISE:
        return val
    return cm.dither(val, err)

# Include network trained to mimick hardware's current control

NETWORK_PATH = None
OBSERVATION_HISTORY = None

trained_network = None

obs_hist = []

def networkInit():
    if not S2R_NETWORK:
        return
    global trained_network
    trained_network = tf.keras.models.load_model(NETWORK_PATH)

def networkReset(obs):
    if not S2R_NETWORK:
        return
    global obs_hist
    obs_hist = []
    for _ in range(-OBSERVATION_HISTORY, 1):
        hist = [obs[0], obs[0], obs[1], obs[2], obs[3], 0]
        obs_hist.append(hist)
    obs_hist[-1].pop(0)
    obs_hist[-1].pop()
    obs_hist[-2].pop()

def networkStep(obs=None, err=None):
    if not S2R_NETWORK:
        return
    global obs_hist
    
    if None is not obs:
        hist = [obs[0], obs[1], obs[2], obs[3]]
        obs_hist.append(hist)
        obs_hist.pop(0)
    
    if None is not err:
        obs_hist[-2].append(err)

def networkPredictPos(target):
    if not S2R_NETWORK:
        return val
    global obs_hist
    obs_hist[-1].insert(0, target)
    
    p = []
    for d in obs_hist:
        p += d
        
    sample = np.array([p])
    
    prediction = trained_network.predict(sample)
    
    return prediction

def networkPredict(val):
    if not S2R_NETWORK:
        return val
    global obs_hist
    obs_hist.append([val])
    obs_hist.pop(0)
    
    p = []
    for d in obs_hist:
        p += d
        
    sample = np.array([p])
    
    prediction = trained_network.predict(sample)
    
    return prediction
    
# dither mechanical properties
def ditherInertia(botId, dith, damp):
    if not S2R_DITHER:
        return
        
    # dither mass, inertia
    for j in range(1,4):
        dinfo = p.getDynamicsInfo(botId, j)
        mass = cm.dither(dinfo[0], dith)
        x = cm.dither(dinfo[2][0], dith**2)
        y = cm.dither(dinfo[2][1], dith**2)
        moment = (x, y, y)
        damping = 0
        if 1 == j:
            damping = cm.clamp(cm.dither(damp, dith), 0, .001)
        if 3 == j:
            y = cm.dither(dinfo[2][2], dith)
            moment = (x, x, y)       
        p.changeDynamics(botId, j, mass=mass, localInertiaDiagonal=moment, jointDamping=damping)


# Randomly generate cogging torque map
        
def ripple_map():
    if not S2R_RIPPLE:
        return []
    max_ripple = .012
    ripple_delta = .004
    
    pos = np.random.uniform(-max_ripple, max_ripple)
    
    buf = [0] * 360
    buf[0] = pos
    for i in range(1, len(buf)):
        delta = np.random.uniform(-ripple_delta, ripple_delta)
        
        if abs(pos + delta) > max_ripple:
            delta *= -1
        
        pos += delta
        buf[i] = pos
    return buf

rip = None

def rad2rip(rad):
    if not S2R_RIPPLE:
        return 0
    rad = cm.wrapRad(rad)
    deg = int(cm.rad2Deg(rad))
    return rip[deg]


# Randomly sample communication delay

def latency(arm_angle, last_arm_angle, pole_angle, last_pole_angle, step):
    setDelay = fetchSetDelay()
    getDelay = fetchGetDelay()
    nnDelay = cm.dither(0.00175, 0.2) # vary prediction time by +/- 20%
    
    if not S2R_LATENCY:
        nnDelay =  0

    pole_angle_diff = cm.difRads(last_pole_angle, pole_angle)
    arm_angle_diff = cm.difRads(last_arm_angle, arm_angle)
    
    pole_delay = getDelay/4 + nnDelay + setDelay/2
    arm_delay = getDelay*(3/4) + nnDelay + setDelay/2
    
    pole_delta = pole_angle_diff * (pole_delay / step)
    arm_delta = arm_angle_diff * (arm_delay / step)
            
    pole_angle_jittered = cm.rad2Norm(pole_angle - pole_delta)
    arm_angle_jittered = cm.rad2Norm(arm_angle - arm_delta)
    
    pole_vel_jittered = (pole_angle_diff - pole_delta) / (step - pole_delay)
    arm_vel_jittered = (arm_angle_diff - arm_delta) / (step - arm_delay)
    
    return [arm_angle_jittered, arm_vel_jittered, pole_angle_jittered, pole_vel_jittered]

def fetchSetDelay():
    if not S2R_LATENCY:
        return 0
    return cd.SET_DELAY[np.random.randint(0, len(SET_DELAY))]

def fetchGetDelay():
    if not S2R_LATENCY:
        return 0
    return cd.GET_DELAY[np.random.randint(0, len(GET_DELAY))]
