from stable_baselines import PPO2

from furuta_env_t_ppo import FurutaEnvTorquePpo2
import common as cm

POLICY_PATH = "policy/"

    
def main():
    env = FurutaEnvTorquePpo2(cm.RUN, render=True)
    #env.setRender(True)
    model = PPO2.load(POLICY_PATH + "ppo2_policy_nn.zip")

    while True:
        obs, done = env.reset(), False
        
        running_reward = 0.0
        ep_len = 0
        for _ in range(100000):
            action, _ = model.predict(obs)
            
            obs, reward, done, infos = env.step(action)
            running_reward += reward
            ep_len += 1
            
            if done:
                print("Episode Reward: %.2f" % (running_reward))
                print("Episode Length: %d" % ep_len)
                running_reward = 0.0
                ep_len = 0
                break


if __name__ == '__main__':
    main()
