import time

from stable_baselines.deepq import DQN

import furuta_env_t_deepq as fed
import common as cm

POLICY_PATH = "policy/"
ARM_TARGET_RAD = cm.deg2Rad(0)


def speedCheck(obs):
    if abs(obs[1]) > cm.VEL_MAX_ARM or abs(obs[3]) > cm.VEL_MAX_POLE:
        print("Overspeed!")

def mainBal():
    env = fed.FurutaEnvTorqueDeepqBal(cm.RUN, render=True) 
    #env.setRender(True)
    model = DQN.load(POLICY_PATH + "deepq_policy_bal.zip", env)
    
    while True:
        obs, done = env.reset(), False
        obs[4] = ARM_TARGET_RAD
        episode_rew = 0
        count = 0
        while not done:
            action, _ = model.predict(obs)
            obs, rew, done, _ = env.step(action)
            obs[4] = ARM_TARGET_RAD
            speedCheck(obs)
            episode_rew += rew
            count += 1
        print("Episode average reward: %.3f" % (episode_rew/count))

    
def mainUp():
    env = fed.FurutaEnvTorqueDeepqUp(cm.RUN, render=True) 
    #env.setRender(True)
    model = DQN.load(POLICY_PATH + "deepq_policy_up.zip", env)
    
    while True:
        obs, done = env.reset(), False
        episode_rew = 0
        count = 0
        while not done:
            action, _ = model.predict(obs)
            obs, rew, done, _ = env.step(action)
            speedCheck(obs)
            episode_rew += rew
            count += 1
        print("Episode average reward: %.3f" % (episode_rew/count))


# run bal and up policies, switching as appropriate
def mainHybrid():
    env = fed.FurutaEnvTorqueDeepq(cm.RUN, render=True) 
    #env.setRender(True)
    modelBal = DQN.load(POLICY_PATH + "deepq_policy_bal_nn.zip", env)
    modelUp = DQN.load(POLICY_PATH + "deepq_policy_up_nn.zip", env)

    while True:
        obs, done = env.reset(), False
        episode_rew = 0
        while not done:
            if abs(obs[2]) > cm.deg2Rad(cm.ANGLE_TERMINAL_MIN_D):
                action, _ = modelUp.predict(obs)
            else:
                action, _ = modelBal.predict(obs)
                
            obs, rew, done, _ = env.step(action)
            
            speedCheck(obs)
            episode_rew += rew
        print("Episode reward: %.3f" % (episode_rew))

if __name__ == '__main__':
    #mainBal()
    #mainUp()
    mainHybrid()
