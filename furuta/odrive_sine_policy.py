import math
import time

import numpy as np

import common as cm

'''
This class is intended as a drop in replacement for an OpenAI policy
for the purpose of driving the motor to collact error data.

stable_baselines.common.BaseRLModel.predict()

'''

CYCLE_POS_MAX = 2 * cm.PI # rads
CYCLE_PERIOD_S = 2

action_table = []

class SinePolicy():
    def __init__(self, throttle_profile):
        print(throttle_profile)
        print(cm.MAX_RADIANS)
        self.predict = self._predictFirst
        
        self.time_start = None
        self.time_last = None
        
        global action_table
        for i in range(len(throttle_profile)):
            a = throttle_profile[i]
            if 0 == a:
                self.action_last = i
            action_table.append(a*cm.MAX_RADIANS)
        print(action_table)
        for a in action_table:
                print(cm.deg2Norm(cm.rad2Deg(a)))
        
    def _predictFirst(self, obs):
        self.time_start = time.time() 
        self.time_last = self.time_start
        self.predict = self._predict
        return self._predict(obs)

    def _predict(self, obs):
        now = time.time()
        
        ### time sine - buggy, not actually sine output
        #print(now - self.time_last)
        #delta = timeSine(time.time() - self.time_start, cm.norm2Rad(obs[0]))        
        #delta = positionSine(cm.norm2Rad(obs[0]))        
        #action = computeAction(delta)
        
        ### random output
        action = randTraj(self.action_last)
        
        ### linear output
        #target = cm.wrapRad((now - self.time_start) * 2 * cm.PI)
        #delta = cm.difRads(cm.norm2Rad(obs[0]), target)
        #action = computeAction(delta)
        
        # decoded action, normalized degrees
        #activation = action_table[action]
        #stepDeg = cm.rad2Deg(activation)
        #stepDegNorm = cm.deg2Norm(stepDeg)
        
        #print("pos: %.3f\ttarg: %.3f\tdelt: %.3f\tact: %.3f" % (cm.rad2Deg(cm.norm2Rad(obs[0])), cm.rad2Deg(target), cm.deg2Norm(cm.rad2Deg(delta)), stepDegNorm) )
        
        self.action_last = action
        self.time_last = now
        return action, None
        
    
#################################

## Util Functions

# pos in radians
def timeLinear(start, now, pos):
    target = cm.wrapRad((now - start) * 2 * cm.PI)
    return cm.difRads(pos, target)
    
def randTraj(action):
    if np.random.uniform(0,1) > 0.0:
        action += cm.sgn() * round(np.random.uniform(0, 1)**3 * 4, 0)
    action = cm.clamp(action, 0, len(action_table) - 1)
    
    return int(action)

def positionSine(pos):
    angle = math.asin(pos/CYCLE_POS_MAX)
    angle += (cm.STEP/CYCLE_PERIOD_S) * 2 * cm.PI
    return angle2Delta(angle, pos)

# position in radians
def timeSine(duration, pos):
    angle = duration/CYCLE_PERIOD_S * 2 * cm.PI
    return angle2Delta(angle, pos)

def angle2Delta(angle, pos):
    target = math.sin(angle) * CYCLE_POS_MAX
    delta = cm.difRads(pos, target)
    return delta

# provide intended displacement in radians
def computeAction(delta):
    action = len(action_table) - 1
    for i in range(len(action_table)):
        if delta < action_table[i]:
            action = i
            if action_table[i] <= 0:
                if i > 0:
                    action = i - 1                
            break
    return action
            

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    THROTTLE_PROFILE = [-1.0, -0.5, -0.25, -0.125, 0, 0.125, 0.25, 0.5, 1.0]
    
    buf_time = []
    buf_pos = []
    buf_act = []
    buf_d_p = []
    
    p = SinePolicy(THROTTLE_PROFILE)
    
    start = time.time()
    
    time_last = start
    obs = [0]
    count = 0
    while True:
        time_last = time.time()
        if time_last > start + CYCLE_PERIOD_S:
            break
            
        action, _ = p.predict(np.array(obs))    
        activation = THROTTLE_PROFILE[action]
        delta = activation * cm.MAX_RADIANS / 2 # reality factor
        
        print("%d\t%.6f\t%.3f\t%.3f\t%.6f" % (count, obs[0], cm.rad2Deg(obs[0]), activation, delta))
        
        buf_time.append(time_last)
        buf_pos.append(obs[0])
        buf_act.append(activation)
        buf_d_p.append(delta)
        
        pos = cm.wrapRad(cm.norm2Rad(obs[0]) + delta)
        pos = cm.rad2Norm(pos)
        obs[0] = pos  

        count += 1
        
        now = time.time()
        dif = now - time_last
        
        time.sleep(cm.STEP - dif)
        
    plt.plot(buf_time, buf_pos)
    plt.plot(buf_time, buf_act)
    plt.plot(buf_time, buf_d_p)
    plt.show()
    
    
    
    
