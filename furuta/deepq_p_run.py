import sys
import time

from stable_baselines.deepq import DQN

import furuta_env_p_deepq as fep
import common as cm
import odrive_sine_policy as ODSP


TEST = "test"
TEST_COUNT_BAL = 500
TEST_COUNT_UP = 100
TEST_COUNT_HYBRID = 50
TEST_CUTOFF_MIN = 100 # a test that ran at least this many steps is considered a long test, for reporting purposes
TEST_CUTOFF_MAX = 50 # a test that ran less than this many steps is considered a short test, for reporting purposes

POLICY_PATH = "policy/"
ARM_TARGET_RAD = cm.deg2Rad(0)


def speedCheck(obs):
    if abs(obs[1]) > cm.VEL_MAX_ARM or abs(obs[3]) > cm.VEL_MAX_POLE:
        print("Overspeed!")
        return True
    return False
    
def mainBal(arg):
    test = arg == TEST
    
    env = fep.FurutaEnvPosDeepqBal(cm.RUN, render = not test) 
    #env.setRender(not test)
    model = DQN.load(POLICY_PATH + "deepq_p_policy_bal_nn2.zip", env)
    
    buf_rew = []
    test_cutoff_count = 0
    complete_count = 0
    test_count = 0
    total_count = 0
    overspeed = 0
    while True:
        test_count += 1
        if test and test_count >= TEST_COUNT_BAL:
            print("\n***Average reward: %.3f\tLong runs: %d\tAverage count: %.3f\tCompleted: %d***\n" % (sum(buf_rew)/float(len(buf_rew)), test_cutoff_count - overspeed, total_count/float(test_count), complete_count))
            break
        
        obs, done = env.reset(), False
        #obs[4] = ARM_TARGET_RAD
        episode_rew = 0
        count = 0
        while not done:
            action, _ = model.predict(obs)
            obs, rew, done, _ = env.step(action)
            #obs[4] = ARM_TARGET_RAD
            if speedCheck(obs):
                overspeed += 1
            episode_rew += rew
            count += 1
            total_count += 1
        if count > 999:
            complete_count += 1
        buf_rew.append(episode_rew)
        if test and count >= TEST_CUTOFF_MIN:
            test_cutoff_count += 1
        print("Episode reward: %.3f\tCount: %d" % (episode_rew, count))

    
def mainUp(arg):
    test = arg == TEST
    
    env = fep.FurutaEnvPosDeepqUp(cm.RUN, render = not test) 
    #env.setRender(not test)
    model = DQN.load(POLICY_PATH + "deepq_p_policy_up_nn2.zip", env)
    
    buf_rew = []
    test_cutoff_count = 0
    test_count = 0
    overspeed = 0
    total_count = 0
    while True:
        test_count += 1
        if test and test_count >= TEST_COUNT_UP:
            print("\n***Average reward: %.3f\tAverage count: %.3f\tShort runs: %d" % (sum(buf_rew)/float(len(buf_rew)), total_count/float(test_count), test_cutoff_count - overspeed))
            break
        
        obs, done = env.reset(), False
        episode_rew = 0
        count = 0
        while not done:
            action, _ = model.predict(obs)
            obs, rew, done, _ = env.step(action)
            if speedCheck(obs):
                overspeed += 1
            episode_rew += rew
            count += 1
            total_count += 1
        buf_rew.append(episode_rew)
        if test and count <= TEST_CUTOFF_MAX:
            test_cutoff_count += 1
        print("Episode reward: %.3f\tCount: %d" % (episode_rew, count))


# run bal and up policies, switching as appropriate
def mainHybrid(arg):
    test = arg == TEST
    
    env = fep.FurutaEnvPosDeepq(cm.RUN, render = not test) 
    #env.setRender(not test)
    modelBal = DQN.load(POLICY_PATH + "deepq_p_policy_bal_nn2.zip", env)
    modelUp = DQN.load(POLICY_PATH + "deepq_p_policy_up_nn2.zip", env)

    buf_rew = []
    test_cutoff_count = 0
    test_count = 0
    complete_count = 0
    overspeed = 0
    while True:
        test_count += 1
        if test and test_count >= TEST_COUNT_HYBRID:
            print("\n***Average reward: %.3f\tLong runs: %d\tComplete: %d" % (sum(buf_rew)/float(len(buf_rew)), test_cutoff_count - overspeed, complete_count))
            break
        
        obs, done = env.reset(), False
        episode_rew = 0
        count = 0
        while not done:
            if abs(obs[2]) > cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D):
                action, _ = modelUp.predict(obs)
            else:
                action, _ = modelBal.predict(obs)
                
            obs, rew, done, _ = env.step(action)
            
            if speedCheck(obs):
                overspeed += 1
            episode_rew += rew
        if count > 999:
            complete_count += 1
        buf_rew.append(episode_rew)
        if test and count <= TEST_CUTOFF_MAX:
            test_cutoff_count += 1
        print("Episode reward: %.3f" % (episode_rew))


def mainTest(arg):
    test = arg == TEST
    
    env = fep.FurutaEnvPosDeepqTest(cm.RUN, render = not test) 
    #env.setRender(not test)
    model = DQN.load(POLICY_PATH + "deepq_p_policy_test_nn.zip", env)
    #model = ODSP.SinePolicy(fep.POSITION_PROFILE)

    buf_rew = []
    test_cutoff_count = 0
    test_count = 0
    complete_count = 0
    while True:
        test_count += 1
        if test and test_count >= TEST_COUNT_HYBRID:
            print("\n***Average reward: %.3f\tLong runs: %d\tComplete: %d" % (sum(buf_rew)/float(len(buf_rew)), test_cutoff_count - overspeed, complete_count))
            break
        
        obs, done = env.reset(), False
        episode_rew = 0
        count = 0
        overspeed = 0
        while not done:
            action, _ = model.predict(obs)
            
            obs, rew, done, _ = env.step(action)
            
            if speedCheck(obs):
                overspeed += 1
            episode_rew += rew
        if count > 999:
            complete_count += 1
        buf_rew.append(episode_rew)
        if test and count <= TEST_CUTOFF_MAX:
            test_cutoff_count += 1
        print("Episode reward: %.3f" % (episode_rew))

if __name__ == '__main__':
    arg = None
    if len(sys.argv) > 1:
        arg = sys.argv[1]
    #mainTest(arg)
    #mainBal(arg)
    #mainUp(arg)
    mainHybrid(arg)
