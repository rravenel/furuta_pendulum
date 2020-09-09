import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
from os import path
import random

# max torque applied to env
# used to scale self.max_torque
#T_MAX = .025
T_MAX = .055

# max applied torque 
RIPPLE_MAX = .012

# odds of applying a ripple torque
RIPPLE_K = 0.5

class PendulumEnv(gym.Env):
    metadata = {
        'render.modes' : ['human', 'rgb_array'],
        'video.frames_per_second' : 30
    }

    def __init__(self, g=10):
        self.x(9.8)
        
        # add self.max_turque so model has awareness of past
        # use to reduce torque reversal, smooth out performance
        high = np.array([1., 1., self.max_speed, self.max_torque])
        #high = np.array([1., 1., self.max_speed])
        self.action_space = spaces.Box(low=-self.max_torque, high=self.max_torque, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)
        self.last_u = 0.
        self.seed()
        
        self.eps = 0
        self.steps = 0
        
    def x(self, g):
        self.max_speed=8
        self.max_torque=2.
        self.dt=.05
        self.g = g
        #self.m = 0.034
        #self.l = 0.172
        self.m = 0.041
        self.l = 0.390
        self.friction = .01
        self.viewer = None

        self.c_th = 1
        self.c_thdot = 0.1
        self.c_u = .1
        self.c_ud = 3/T_MAX
        self.c_r = .5

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self,u):
        self.steps += 1
        u0 = np.clip(u, -self.max_torque, self.max_torque)[0]
        u = (u0/self.max_torque) * T_MAX
        th, thdot = self.state # th := theta

        g = self.g
        m = self.m
        l = self.l
        f = self.friction
        dt = self.dt

        reverse = 1 if np.sign(u) != np.sign(self.last_u) else 0
        #costs = angle_normalize(th)**2 + .1*thdot**2 + .001*(u**2)
        pos = angle_normalize(th)
        costs = self.c_th*pos**2 + self.c_thdot*thdot**2 + self.c_u*(u**2) + self.c_r*reverse + self.c_ud*abs(u0 - self.last_u)
        #costs = self.c_th*pos*np.sign(pos) + self.c_thdot*thdot**2 + self.c_u*(u**2) + self.c_r*reverse

        #print("cost: %.4f\tpos: %.4f\tvel: %.4f\tu: %.4f" % (-costs, pos, thdot, self.last_u))        
        
        ur = ripple(u)

        a_g = -3*g/(2*l) * np.sin(th + np.pi)
        a_u = 3./(m*l**2)*ur
        a_f = -np.sign(thdot)*f

        #newthdot = thdot + (-3*g/(2*l) * np.sin(th + np.pi) + 3./(m*l**2)*u) * dt
        newthdot = thdot + (a_g + a_u + a_f) * dt
        
        newth = th + newthdot*dt
        clipthdot = np.clip(newthdot, -self.max_speed, self.max_speed) #pylint: disable=E1111
        if clipthdot < newthdot:
            #print("Clipped: %f" % (newthdot))
            pass
        newthdot = clipthdot
        
        self.last_u = u0 # for rendering
        self.state = np.array([newth, newthdot])
        return self._get_obs(), -costs, False, {}

    def reset(self):
        high = np.array([np.pi, 1])
        #high = np.array([np.pi, 0])
        self.state = self.np_random.uniform(low=-high, high=high)
        #self.state = np.array([np.pi, 0])
        self.last_u = 0
        self.eps += 1
        
        #print("eps: %d\tsteps/ep: %d" % (self.eps, self.steps))
        self.steps = 0
        
        #self.migrate()
        
        return self._get_obs()
    
    # migrate pendulum dimensions from default to physical
    def migrate(self):
        if self.eps > 50 and self.eps <= 200:
            self.c_u += .09/150
            self.max_torque -= 1.9/150
            self.m -= (1 - .034)/150
            self.l -= (1 - .172)/150

    def _get_obs(self):
        theta, thetadot = self.state
        return np.array([np.cos(theta), np.sin(theta), thetadot, self.last_u])
        #return np.array([np.cos(theta), np.sin(theta), thetadot])

    def render(self, mode='human'):

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(500,500)
            self.viewer.set_bounds(-2.2,2.2,-2.2,2.2)
            rod = rendering.make_capsule(1, .2)
            rod.set_color(.8, .3, .3)
            self.pole_transform = rendering.Transform()
            rod.add_attr(self.pole_transform)
            self.viewer.add_geom(rod)
            axle = rendering.make_circle(.05)
            axle.set_color(0,100,0)
            self.viewer.add_geom(axle)
            fname = path.join(path.dirname(__file__), "assets/clockwise.png")
            self.img = rendering.Image(fname, 1., 1.)
            self.imgtrans = rendering.Transform()
            self.img.add_attr(self.imgtrans)

        self.viewer.add_onetime(self.img)
        self.pole_transform.set_rotation(self.state[0] + np.pi/2)
        if self.last_u:
            #self.imgtrans.scale = (-self.last_u/2, np.abs(self.last_u)/2)
            #self.imgtrans.scale = (-self.last_u/self.max_torque, np.abs(self.last_u)/self.max_torque)
            self.imgtrans.scale = (-self.last_u/T_MAX, np.abs(self.last_u)/T_MAX)

        return self.viewer.render(return_rgb_array = mode=='rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None

def angle_normalize(x):
    return (((x+np.pi) % (2*np.pi)) - np.pi)
    
def ripple(u):
    if random.random() < RIPPLE_K:
        u += random.uniform(-RIPPLE_MAX, RIPPLE_MAX)
    return u