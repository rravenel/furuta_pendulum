import numpy as np
import odrive
import random
import time

'''
Random controller for physical pendulum
'''

cpr = 8192
p0 = 0
t_run = 5
c_max = 3.0
v_max = 3 * cpr
dt = 0.05

def p2r(p):
	return 2 * np.pi * (p/cpr)

def v2rs(v):
	return p2r(v)

# copied from gym env for model continuity
# it handles wrap, turning pi in to -pi
def angle_normalize(x):
	x = p2r(x)
	return (((x+np.pi) % (2*np.pi)) - np.pi)

def action_rand():
	return random.uniform(-c_max, c_max)

print("Connecting...")
d = odrive.find_any()
print("Connected")
x = d.axis0

x.controller.config.control_mode = 3
x.controller.pos_setpoint = p0
x.controller.config.control_mode = 1

t_start = time.time()
t_last = t_start

while t_last - t_start < t_run:
	t_now = time.time()
	t_diff = t_now - t_last
	if t_diff < dt:
		time.sleep(dt - t_diff)

	p = x.encoder.pos_cpr
	v = x.encoder.vel_estimate
	if abs(v) > v_max:
		print("Max velocity exceeded: %f" % (v))
		c = 0
	else:
		c = action_rand()
	
	x.controller.current_setpoint = c
	
	t_last = t_now

x.controller.current_setpoint = 0