from __future__ import absolute_import, division, print_function, unicode_literals

import json
import matplotlib.pyplot as plt
import numpy as np
import odrive
import tensorflow as tf
import time

cpr = 8192
v_lim = 400000
#v_target = 50000
v_target = 0
#v_roll_ave = 5
v_roll_ave = 1
c_lim = 10
#c_launch = 1.3
c_launch = .4
t_run = 5
vel_max = 600000
cur_max = 10.000001
T_max = 1000

# torques normalized between 0 and 1 with a zero value at 0.5
#T_target = 0.5039
#T_target = 0.5045
T_target = 0.505


#model = tf.keras.models.load_model("data/tmp/model_tmp.h5")
model = tf.keras.models.load_model("data/model1.h5")

print("Connecting...")
d = odrive.find_any()
print("Connected")
x = d.axis0

t_buf = []
v_buf = []
p_buf = []

v_his = [v_target] * v_roll_ave

v_x_lim = 3
v_x_buf = []
p_last =  x.encoder.pos_cpr
t_start = time.time()
t_last = t_start

print("Start loop...")
#x.controller.config.control_mode = 2
#x.controller.vel_setpoint = v_target
x.controller.config.control_mode = 1
x.controller.current_setpoint = c_launch
time.sleep(0.1)
#x.controller.config.control_mode = 1
while t_last - t_start < t_run:
	p = x.encoder.pos_cpr

	t = time.time()
	t_diff = t - t_last

	p_diff = p - p_last
	if abs(p_diff) > cpr/2:
		if p_diff < 0:
			p_diff += cpr
		else:
			p_diff -= cpr	

	v = (p_diff) / t_diff
	print("p: %d\tp_diff: %d\tv: %d" % (int(p), int(p_diff), int(v)))
	
	if v > v_lim or v < v_lim * -1:
		v_x_buf.append(v)
		if len(v_x_buf) > v_x_lim:
			x.controller.current_setpoint = 0
			print("			Overspeed - abort. %.f" % (sum(v_x_buf)/len(v_x_buf)))
			break
	else:
		v_x_buf = []

	p_last = p
	t_last = t

	t_buf.append(t)
	v_his.pop(0)
	v_his.append(v)
	v_buf.append(sum(v_his)/len(v_his))
	p_buf.append(p + 50000)

	#v = v_target
	p_ = p/cpr
	v_ = (1 + v/vel_max)/2
	t_ = T_target

	buf = []
	buf.append(p_)
	buf.append(v_)
	buf.append(t_)

	sample = np.array([buf])
	pred = model.predict(sample)
	c_set = pred[0][0]
	#c_set = (c_set - 0.5) * 2 * cur_max * -1
	c_set = (c_set - 0.5) * 2 * cur_max

	if c_set > c_lim or c_set < c_lim * -1:
		print("			Overcurrent prediction: %f" % (c_set))
		c_set = c_lim
	
	x.controller.current_setpoint = c_set 

x.controller.current_setpoint = 0

#plt.plot(t_buf, p_buf)
plt.plot(t_buf, v_buf)
plt.show()