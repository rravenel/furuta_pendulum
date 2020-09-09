import datetime
import time

from stable_baselines import TRPO
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common import make_vec_env

from furuta_env_p_trpo import FurutaEnvPosTrpoUp, FurutaEnvPosTrpoBal
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
    model = TRPO(
        env=env,
        policy=MlpPolicy,
        policy_kwargs=dict(net_arch=arch),
        n_cpu_tf_sess=None
    )
        
    # train the agent on the environment
    model.learn(
        total_timesteps=steps,
        log_interval=10,
        #log_dir=".",
        #record_video=False
    )

    # save trained model
    model.save(POLICY_PATH + file, cloudpickle=True)
    print("Duration: %.1f" % ((time.time() - start)/60))


def main():
    envBal = make_vec_env(FurutaEnvPosTrpoBal, n_envs=1, env_kwargs={"state": cm.TRAIN, "render":False})
    train(envBal, "trpo_pos_policy_bal.pkl", STEPS, [DIMENSION,DIMENSION])
    
    #envUp = make_vec_env(FurutaEnvPosTrpoUp, n_envs=1, env_kwargs={"state": cm.TRAIN, "render":False})    
    #train(envUp, "trpo_pos_policy_up.pkl", STEPS, [DIMENSION,DIMENSION])
    
if __name__ == '__main__':
    print(datetime.datetime.now())
    main()

