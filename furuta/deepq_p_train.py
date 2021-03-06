import datetime
import time

import gym
import tensorflow as tf

from stable_baselines.deepq import DQN, MlpPolicy

from furuta_env_p_deepq import FurutaEnvPosDeepqBal, FurutaEnvPosDeepqUp, FurutaEnvPosDeepqTest
import common as cm

POLICY_PATH = "policy/"

#STEPS = 1000
#STEPS = 60000
STEPS = 100000
#STEPS = 200000
#DIMENSION = 16
#DIMENSION = 32
#DIMENSION = 64
DIMENSION = 128

class CustomPolicy(MlpPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch,
                 reuse=False, obs_phs=None, dueling=True, **_kwargs):
        super(MlpPolicy, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse,
                                        feature_extraction="mlp", obs_phs=obs_phs, dueling=dueling,
                                        layer_norm=False, **_kwargs, layers=[DIMENSION, DIMENSION])
                                           
def callback(lcl, glb):
    return True

def train(env, fname):
    env.setRender(False)
    env.reset()
    
    start = time.time()
    model = DQN(
        env=env,
        policy=CustomPolicy,
        learning_rate=1e-3,
        buffer_size=50000,
        exploration_fraction=0.1,
        exploration_final_eps=0.02
    )
    model.learn(total_timesteps=STEPS, callback=callback)

    # save trained model
    model.save(POLICY_PATH + fname)
    print("Duration: %.3f" % ((time.time() - start)/60))

def main():
    #train(FurutaEnvPosDeepqTest(cm.TRAIN),"deepq_p_policy_test_nn.zip")
    train(FurutaEnvPosDeepqBal(cm.TRAIN),"deepq_p_policy_bal_nn2.zip")
    #train(FurutaEnvPosDeepqUp(cm.TRAIN),"deepq_p_policy_up_nn2.zip")

if __name__ == '__main__':
    print(datetime.datetime.now())
    main()

