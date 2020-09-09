import json
import math
import numpy as np
import sys
import time

from stable_baselines.deepq import DQN
import odrive

import common as cm

MODE_EVAL = "mode_eval"
MODE_RUN = "mode_run"
MODE_TURBO = "mode_turbo"

#EXECUTION_POLICY = MODE_TURBO # 12amps 1000 steps
#EXECUTION_POLICY = MODE_RUN # 10amps, 1000 steps
EXECUTION_POLICY = MODE_EVAL # 5amps, 500steps

#THROTTLE_PROFILE = [-1.0, -0.5, -0.25, -0.125, 0, 0.125, 0.25, 0.5, 1.0]
#THROTTLE_PROFILE = [-1.0, -0.5, 0, 0.5, 1.0]
THROTTLE_PROFILE = [-1.0, 0, 1.0]

CURRENT_CONTROL = 1

VEL_MAX_COUNT = 3 # number of times measured velocity is too fast before cutting current

if EXECUTION_POLICY is MODE_TURBO:
    MAX_CURRENT = 12.0
    LOOP_COUNT = 1000
elif EXECUTION_POLICY is MODE_RUN:
    MAX_CURRENT = 10.0
    LOOP_COUNT = 1000
else:
    MAX_CURRENT = 5.0
    LOOP_COUNT = 500

ARM_TARGET_RAD = 0

drive = None
axis0 = None
axis1 = None

arm_last = 0
pole_last = cm.PI # pi is straight down
time_last = 0
pole_offset = 0

vel_max_arm_count = 0
vel_max_pole_count = 0

buf_current = [] # tuple(current_command, obs, buf_current_samples)
buf_current_samples = []
current_target = 0
obs_prev = None

buf = []


def getArm():
    return cm.rad2Norm(cm.cpr2Rad(axis0.encoder.pos_cpr))

def getPole():
    return cm.rad2Norm(cm.cpr2Rad(axis1.encoder.pos_cpr))

def getVel(pos, pos_last, time_diff):
    # is there a sign flip?
    if np.sign(pos) == np.sign(pos_last):
        pos_diff = pos - pos_last
    else:
        # identify the center point of the sign flip
        center = 0 if abs(pos_last) < cm.PI/2 else cm.PI
        # magnitude of position change
        pos_diff = abs((abs(pos) - center) + (abs(pos_last) - center))
        
        # determine direction of motion
        vel_sign = 1
        if (0 == center and pos_last > 0) or (cm.PI == center and pos_last < 0):
            vel_sign = -1
        
        pos_diff *= vel_sign
        
    return pos_diff/time_diff
    
def checkArmVel(obs):
    v = obs[1]
    global vel_max_arm_count
    if abs(v) >= cm.VEL_MAX_ARM:
        vel_max_arm_count = vel_max_arm_count + 1
        if vel_max_arm_count >= VEL_MAX_COUNT:
            setCurrent(0, obs)
            print("Arm overspeed!")
            return False
    else:
        if vel_max_arm_count > 0:
            vel_max_arm_count -= 1
    return True
    
def checkPoleVel(obs):
    v = obs[3]
    global vel_max_pole_count
    if abs(v) >= cm.VEL_MAX_POLE:
        vel_max_pole_count = vel_max_pole_count + 1
        if vel_max_pole_count >= VEL_MAX_COUNT:
            setCurrent(0, obs)
            print("\t\t\t\tPole overspeed!")
            return False
    else:
        if vel_max_pole_count > 0:
            vel_max_pole_count -= 1
    return True

def getCurrent_x():
    i = -axis0.motor.current_control.Iq_measured # odrive current sign flip
    buf_current_samples.append((i))
    return i

def getCurrent():
    i = -axis0.motor.current_control.Iq_measured # odrive current sign flip
    return i

def _setCurrent(c):
    axis0.controller.current_setpoint = c

def setCurrent(c, obs):
    global current_target
    current_target = c
    _setCurrent(c)
    
def setCurrent_x(c, obs):
    global buf_current, buf_current_samples, current_target, obs_prev
    buf_current.append({"target":current_target, "obs":obs_prev, "i":buf_current_samples})
    _setCurrent(c)
    current_target = c
    obs_prev = obs
    buf_current_samples = []

def computeCurrent(action):
    return MAX_CURRENT * THROTTLE_PROFILE[action]

def initObs():
    global time_last, arm_last, pole_offset
    
    time_last = time.time()
    arm_last = getArm()
    # pole starts at pi by default; record starting offset
    pole_offset = getPole()

def getObs(render=True):
    global arm_last, pole_last, time_last

    now = time.time()
    diff = now - time_last
    pole = getPole()
    arm = getArm()

    pole += cm.PI - pole_offset
    pole = cm.rad2Norm(pole)

    arm_vel = getVel(arm, arm_last, diff)
    pole_vel = getVel(pole, pole_last, diff)
        
    arm_last = arm
    pole_last = pole
    time_last = now
    
    if render:
        print("arm: %.4f %.4f \tpole: %.4f %.4f" % (arm, arm_vel, pole, pole_vel))
    
    return [arm, arm_vel, pole, pole_vel, ARM_TARGET_RAD]

def setupOdrive():
    global drive, axis0, axis1, ARM_TARGET_RAD
    
    print("Connecting to Odrive...")
    drive = odrive.find_any()
    print("Connected!")
    axis0 = drive.axis0
    axis1 = drive.axis1
    
    axis0.controller.config.control_mode = CURRENT_CONTROL
    _setCurrent(0)

    ARM_TARGET_RAD = cm.deg2Rad(0)
    
# separate policies for spin-up, balance; torque control; arm target
def main():
    setupOdrive()
    # sanity check
    if drive is None:
        print("Failed to initialize Odrive.  Exiting...")
        exit()
    _setCurrent(0)

    nnBal = DQN.load("deepq_policy_bal.zip.quiet")
    nnUp = DQN.load("deepq_policy_up.zip.quiet")
    
    initObs()
    obs = getObs()
    
    reward = 0
    i = 0
    while i < LOOP_COUNT:
        i += 1
        mark = time.time()
            
        action = 0
        if checkArmVel(obs) and checkPoleVel(obs):
            policy = None
            if abs(obs[2]) > cm.deg2Rad(10):
                policy = "spin-up"
                action, _ = nnUp.predict(np.array(obs))
            else:
                policy = "balance"
                action, _ = nnBal.predict(np.array(obs))
            
            current = computeCurrent(action)
            print("%s\taction: %d\tcurrent: %.2f" % (policy, action, current))
            setCurrent(current, obs)
        
        diff = time.time() - mark
        render = True
        while diff < LOOP_COUNT:
            obs = getObs(render)
            cur = getCurrent()
            render = False
            buf.append({"target":current_target, "obs":obs, "i":cur})
            diff = time.time() - mark
        
        reward += math.cos(abs(obs[2])) - abs(current/MAX_CURRENT) * 0.001

    print("Episode reward: %.1f\tdata len: %d" % (reward, len(buf_current)))
    _setCurrent(0)
    
    stamp = int(time.time() / 1)
    fname = "data/current_data_3_b_" + str(stamp) + ".json"
    with open(fname, 'w') as f:
        json.dump(buf, f)
    print("Wrote behavioral data to: " + fname)

if __name__ == '__main__':
    try:
        main()
    except:
        _setCurrent(0)
        print("Unknown exception...")
        print(sys.exc_info())
        
    _setCurrent(0)
    print("Done")
    
