import time

import gym
import tensorflow as tf

from stable_baselines.deepq import DQN, MlpPolicy

from furuta_env_t_deepq import FurutaEnvTorqueDeepqBal, FurutaEnvTorqueDeepqUp
import common as cm

#STEPS = 10000
#STEPS = 60000
STEPS = 200000
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
    model.save(fname)
    print("Duration: %.1f" % ((time.time() - start)/60))

def main():
    train(FurutaEnvTorqueDeepqBal(cm.TRAIN),"deepq_policy_bal_nn.zip")    
    #train(FurutaEnvTorqueDeepqUp(cm.TRAIN),"deepq_policy_up_nn.zip")    

if __name__ == '__main__':
    main()

