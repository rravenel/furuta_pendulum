import json
import math
import numpy as np
import sys
import time

from stable_baselines.deepq import DQN
import odrive

import common as cm
import odrive_sine_policy as ODSP

DATA_LOG = True

# action delta of 3
# tf=3 looked pretty good, dynamic, still smooth
# tf=4 bordering on too abrupt

# action delta of 4
# tf=3 

# for data collection
#LOOP_COUNT = 3000
#THROTTLE_FACTOR = 3

LOOP_COUNT = 200
THROTTLE_FACTOR = 3

MAX_RADIANS = cm.MAX_RADIANS * THROTTLE_FACTOR

THROTTLE_PROFILE = [-1.0, -0.5, -0.25, -0.125, 0, 0.125, 0.25, 0.5, 1.0]
#THROTTLE_PROFILE = [-1.0, -0.5, 0, 0.5, 1.0]
#THROTTLE_PROFILE = [-1.0, 0, 1.0]

POLICY_PATH = "policy/"

CURRENT_CONTROL = 1
POSITION_CONTROL = 3

VEL_MAX_COUNT = 1 # number of times measured velocity is too fast before cutting current

ARM_TARGET_RAD = 0

drive = None
axis0 = None
axis1 = None


time_last = None

# All stored angles are in normalized radians
arm_last = None
pole_last = cm.PI # assume we start straight down
pole_offset = None
pos_target = None
target_delta = 0

obs = None

vel_max_arm_count = 0
vel_max_pole_count = 0

D_VEL = "vel"
D_ERR = "err"
buf_hist = [] # dict: vel, err


def getArm():
    return cm.rad2Norm(cm.cpr2Rad(axis0.encoder.pos_cpr))
    
def getPole():
    return cm.rad2Norm(cm.cpr2Rad(axis1.encoder.pos_cpr))
    
def getVel(pos, pos_last, time_diff):
    return cm.difRads(cm.norm2Rad(pos_last), cm.norm2Rad(pos)) / time_diff
    
def checkArmVel():
    global vel_max_arm_count
    v = obs[1]
    if abs(v) >= cm.VEL_MAX_ARM:
        vel_max_arm_count += 1
        if vel_max_arm_count >= VEL_MAX_COUNT:
            reduce(np.sign(v))
            print("\tArm overspeed!")
            return False
    else:
        if vel_max_arm_count > 0:
            vel_max_arm_count -= 1
    return True
    
def checkPoleVel():
    global vel_max_pole_count
    v = obs[3]
    if abs(v) >= cm.VEL_MAX_POLE:
        vel_max_pole_count += 1
        if vel_max_pole_count >= VEL_MAX_COUNT:
            reduce(np.sign(v))
            print("\t\t\t\tPole overspeed!")
            return False
    else:
        if vel_max_pole_count > 0:
            vel_max_pole_count -= 1
    return True

def setCurrent(c):
    axis0.controller.current_setpoint = c
    
def decodeAction(action):
    return THROTTLE_PROFILE[action]

def setPosition(rads):
    global pos_target
    pos = cm.rad2Cpr(rads)
    axis0.controller.pos_setpoint = pos
    pos_target = cm.rad2Norm(rads)
    
def calculateTarget(delta):
    return cm.wrapRad(cm.norm2Rad(arm_last) + delta)

def act(action):
    global target_delta
    activation = decodeAction(action)
    target_delta = activation * MAX_RADIANS
    setPosition(calculateTarget(target_delta))
    print("Activation: %.3f\tDelta: %.6f\tTarget: %.6f" % (activation, target_delta, pos_target))
    
def reduce(sign):
    global target_delta
    target_delta = sign * MAX_RADIANS / 2
    setPosition(calculateTarget(target_delta))
    print("Delta: %.6f\tTarget: %.6f" % (target_delta, pos_target))
        
def deactivateMotor():
    axis0.controller.config.control_mode = CURRENT_CONTROL
    setCurrent(0)
    
def initObs():
    global time_last, arm_last, pole_last, pole_offset
    
    time_last = time.time()
    arm_last = getArm()
    pole_offset = cm.difRads(cm.norm2Rad(getPole()), cm.norm2Rad(pole_last))
    
    # sometimes twitches on connect - wait for settle
    updateObs()
    while 0 != obs[3]:
        time.sleep(0.5)
        updateObs()

def updateObs(render=True):
    global arm_last, pole_last, time_last, obs

    now = time.time()
    diff = now - time_last
    pole = getPole()
    arm = getArm()

    pole = cm.rad2Norm(cm.wrapRad(cm.norm2Rad(pole) + cm.norm2Rad(pole_offset)))
            
    arm_vel = getVel(arm, arm_last, diff)
    pole_vel = getVel(pole, pole_last, diff)
      
    arm_last = arm
    pole_last = pole
    time_last = now
    
    if render:
        print("arm: %.6f %.6f \tpole: %.6f %.6f" % (arm, arm_vel, pole, pole_vel))
        print("arm: %.3f %.3f \t\tpole: %.3f %.3f" % (cm.rad2Deg(cm.norm2Rad(arm)), np.sign(arm_vel)*cm.rad2Deg(abs(arm_vel)), cm.rad2Deg(cm.norm2Rad(pole)), np.sign(pole_vel)*cm.rad2Deg(abs(pole_vel))))
        
    obs = [arm, arm_vel, pole, pole_vel, ARM_TARGET_RAD]
    return obs

def setupOdrive():
    global drive, axis0, axis1, ARM_TARGET_RAD
    
    print("Connecting to Odrive...")
    drive = odrive.find_any()
    print("Connected!")
    axis0 = drive.axis0
    axis1 = drive.axis1
    
    axis0.controller.config.control_mode = POSITION_CONTROL

    ARM_TARGET_RAD = cm.deg2Rad(0)
    
# separate policies for spin-up, balance; position control; arm target
def main():
    setupOdrive()
    # sanity check
    if drive is None:
        print("Failed to initialize Odrive.  Exiting...")
        exit()

    #nnBal = DQN.load(POLICY_PATH + "deepq_p_policy_bal.zip")
    #nnUp = DQN.load(POLICY_PATH + "deepq_p_policy_up.zip")
    
    nnBal = ODSP.SinePolicy(THROTTLE_PROFILE)
    nnUp = nnBal

    initObs()
    
    max_arm_vel = 0
    max_pole_vel = 0
    speedCheck = 0
    reward = 0
    i = 0
    while i < LOOP_COUNT:
        i += 1
        print("====================Step: %d=============================" % (i))
        
        prev_arm = arm_last
        
        diff = time.time() - time_last
        if 0 < diff and diff < cm.STEP:
            time.sleep(cm.STEP - diff)
        updateObs()
        
        if abs(obs[1]) > max_arm_vel:
            max_arm_vel = abs(obs[1])
        if abs(obs[3]) > max_pole_vel:
            max_pole_vel = abs(obs[3])
        
        if abs(obs[2]) > cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D):
            policy = nnUp
        else:
            policy = nnBal

        activation = 0
        prev_pos_target = pos_target
        prev_target_delta = target_delta
        if checkArmVel() and checkPoleVel():
            action, _ = policy.predict(np.array(obs))
            act(action)        
        else:
            speedCheck += 1

        if prev_pos_target is not None:
            delta = cm.difRads(cm.norm2Rad(prev_arm), cm.norm2Rad(obs[0]))
            pos_err = delta - prev_target_delta
            buf_hist.append({D_VEL:obs[1], D_ERR:pos_err})
            print("Error: %.6f\t%.3f" % (pos_err, cm.rad2Deg(pos_err)))

        reward += math.cos(abs(obs[2]))

    deactivateMotor()

    print("Episode reward: %.1f\tdata len: %d" % (reward, len(buf_hist)))
    print("max_arm_vel-d: %.3f\tmax_pole_vel-d: %.3f\tspeedCheck: %d" % (cm.rad2Deg(max_arm_vel), cm.rad2Deg(max_pole_vel), speedCheck))
    
    if DATA_LOG:
        stamp = int(time.time() / 1)
        #fname = "data/deepq_p_data_" + str(stamp) + ".json"
        fname = "data/odsp_data_" + str(stamp) + ".json"
        with open(fname, 'w') as f:
            json.dump(buf_hist, f)
        print("Wrote behavioral data to: " + fname)


if __name__ == '__main__':
    try:
        main()
    except:
        print("Unknown exception...")
        print(sys.exc_info())
        deactivateMotor()
    
    deactivateMotor()
    print("Done")
    
