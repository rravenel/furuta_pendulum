import gym
import numpy as np

from gym.wrappers import FlattenDictWrapper

env = gym.make('Pendulum-v0') 
env.reset()
#env.state = np.array([-np.pi, 0])

for _ in range(250):
    env.render()
    _, _, done, _ = env.step(env.action_space.sample())
    #_, _, done, _ = env.step([env.max_torque])
    #_, _, done, _ = env.step([0])
    
env.close()