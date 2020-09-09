import datetime
import time

from stable_baselines import PPO1
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common import make_vec_env

from furuta_env_p_ppo import FurutaEnvPosPpoUp, FurutaEnvPosPpoBal
import common as cm

POLICY_PATH = "policy/"

#STEPS = 10000
#STEPS = 60000
#STEPS = 100000
#STEPS = 200000
#STEPS = 500000
STEPS = 2500000
#DIMENSION = 16
#DIMENSION = 32
#DIMENSION = 64
DIMENSION = 128

def callback(lcl, glb):
    return False

def train(env, file, steps, arch):
    start = time.time()
    #env.setRender(False)
    
    # create the learning agent
    model = getPpo1(env, arch)
        
    # train the agent on the environment
    model.learn(
        #total_timesteps=10000000,
        total_timesteps=steps,
        log_interval=10,
        #log_dir=".",
        #record_video=False
    )

    # save trained model
    model.save(POLICY_PATH + file, cloudpickle=True)
    print("Duration: %.1f" % ((time.time() - start)/60))

def getPpo1(env, arch):
    return PPO1(
        env=env,
        policy=MlpPolicy,
        policy_kwargs=dict(net_arch=arch),
        n_cpu_tf_sess=None
    )

def getPpo2(env, arc):
    return PPO2(
        #tensorboard_log=saver.data_dir,
        policy=MlpPolicy,
        #policy_kwargs=dict(net_arch=[dict(pi=[DIMENSION, DIMENSION], vf=[DIMENSION, DIMENSION])]),
        policy_kwargs=dict(net_arch=arch),
        env=env,
        gamma=0.998,
        n_steps=1000,
        ent_coef=0,
        learning_rate=1e-3,
        vf_coef=0.5,
        max_grad_norm=0.5,
        lam=0.95,
        nminibatches=10,
        noptepochs=10,
        cliprange=0.2,
        verbose=1,
    )

def main():
    envBal = make_vec_env(FurutaEnvPosPpoBal, n_envs=1, env_kwargs={"state": cm.TRAIN, "render":False})
    train(envBal, "ppo1_pos_policy_bal.pkl", STEPS, [DIMENSION,DIMENSION])
    
    #envUp = make_vec_env(FurutaEnvPosPpoUp, n_envs=1, env_kwargs={"state": cm.TRAIN, "render":False})    
    #train(envUp, "ppo1_pos_policy_up.pkl", STEPS, [DIMENSION,DIMENSION])
    
if __name__ == '__main__':
    print(datetime.datetime.now())
    main()

