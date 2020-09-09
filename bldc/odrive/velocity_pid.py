import  json
import matplotlib.pyplot as plt
import numpy as np
import odrive
import time

# velocity PID controller

'''
p: 0.080000	i: 0.012000	d: 0.012000	r: 0.000000	threshold: 10	anti_ripple: 0
Connecting...
Connected!
Loop freq.: 225.830078Hz
std c: 1568.547296	std v: 1111.231645
max c: 10398.316341	mean: 1219.847200		max v: 11500.000977	mean: 9705

p: 0.080000	i: 0.012000	d: 0.012000	r: 0.000000	threshold: 10	anti_ripple: 0
Connecting...
Connected!
Loop freq.: 225.769043Hz
std c: 896.060882	std v: 442.692430
max c: 3388.145270	mean: 794.037284		max v: 2500.000244	mean: 984

p: 0.080000	i: 0.007000	d: 0.009000	r: 0.000000	threshold: 10	anti_ripple: 0
Connecting...
Connected!
Loop freq.: 153.808594Hz
std c: 597.658625	std v: 492.960539
max c: 1948.304413	mean: 615.204410		max v: 2500.000244	mean: 993

'''

cpr = 8192
vel_max = 300000
cur_max = 12
vel_threshold = 10
anti_ripple = False

vel_target = 200
t_run = (cpr / vel_target) * 1

p_gain = 0.08
i_gain = 0.012
d_gain = 0.012
r_gain = 0

#p_gain = 0.02
#i_gain = 0.001
#d_gain = 0.01
#r_gain = 0.2


print("p: %f\ti: %f\td: %f\tr: %f\tthreshold: %d\tanti_ripple: %i" % (p_gain, i_gain, d_gain, r_gain, vel_threshold, anti_ripple))

def overspeed(arg):
	return arg > vel_max

def overcurrent(arg):
	return arg > cur_max

def cpr2rad(arg):
	return 2 * np.pi * (arg / cpr)

map = [0] * cpr

def round(x):
	i = int(x)
	dec = x - i
	if dec >= 0.5:
		return i+1
	return i


def next_non_zero(start):
	for i in range(start+1, cpr):
		if 0 != map[i]:
			return i
	return -1

def interpolate(input):
	# build sparse map
	for d in input:
		p = d[0]
		c = d[1] * -1

		p = round(p)
		map[p] = c

	# find first value
	first = next_non_zero(0)
	last = first
	next = next_non_zero(last)
	while next > 0:
		c_last = map[last]
		c_next = map[next]

		diff = next - last
		# 'next' pos already has a value, so don't count it
		diff -= 1

		if 1 == diff:
			map[last + 1] = c_next
		else:
			half = int(diff/2)

			for i in range(half):
				map[last + 1 + i] = c_last
	
			i = last + 1 + half
			while i < next:
				map[i] = c_next
				i += 1

		last = next
		next = next_non_zero(last)
		
	# fill in leading 0's
	c_leading = map[first]
	c_trailing = map[last]

	leading = first
	trailing = cpr - 1 - last
		
	diff = leading + trailing
	half = int(diff/2)

	for i in range(half):
		index = last + 1 + i
		if index > cpr - 1:
			index -= (cpr)
		map[index] = c_last

	i = last + 1 + half
	if i > cpr - 1:
		i -= (cpr)
	
	while i != first:
		map[i] = c_next
		i += 1
		if i > cpr - 1:
			i -= (cpr)

data = None
with open("data/pos_cur_b.json", 'r') as file:
	data = json.load(file)

data = data["forward"]
interpolate(data)


print("Connecting...")
d = odrive.find_any()
print("Connected!")
x = d.axis0

x.controller.config.control_mode = 1
x.controller.current_setpoint = 0

v_plt = []
c_plt = []
t_plt = []

buf_err = [0]
err_last = 0
c = 0

t_start = time.time()
t_last = t_start
time.sleep(0.01)
c_rip_last = 0
count = 0
while t_last - t_start < t_run:
	t_now = time.time()
	v = x.encoder.vel_estimate
	
	if overspeed(v):
		print("Overspeed!")
		x.controller.current_setpoint = 0
		break

	v_err = cpr2rad( vel_target - v)

	p_val = v_err * p_gain

	if abs(v_err) < vel_threshold:
		buf_err = [0]

	buf_err.append(v_err)
	i_val = sum(buf_err) * i_gain
	
	err_diff = v_err - err_last
	t_diff = t_now - t_last
	d_val = (err_diff / t_diff) * d_gain

	c += p_val + i_val + d_val
	if anti_ripple:
		pos = x.encoder.pos_cpr
		pr = round(pos)
		if pr > cpr - 1:
			pr -= cpr
		c_rip = map[pr] * -1
		c += (c_rip - c_rip_last) * r_gain

	if overcurrent(c):
		print("Overcurrent! %f" % (c))
		x.controller.current_setpoint = 0
		break

	x.controller.current_setpoint = c

	t_last = t_now
	err_last = v_err

	v_plt.append(v)
	c_plt.append(c * 1000)
	t_plt.append(t_now - t_start)
	count += 1


x.controller.current_setpoint = 0
x.controller.config.control_mode = 3

print("Loop freq.: %fHz" % (count / t_run))

print("std c: %f\tstd v: %f" % (np.std(c_plt), np.std(v_plt)))

def max_abs(arg):
	buf = [0]
	zx = 0
	for val in arg:
		if val >= 0 != buf[-1] > 0:
			zx += 1 
		v_a = abs(val)
		buf.append(v_a)

	return max(buf), np.mean(buf)

c_max, c_mean = max_abs(c_plt)
v_max, v_mean = max_abs(v_plt)

print("max c: %f\tmean: %f\t\tmax v: %f\tmean: %d" % (c_max, c_mean, v_max, np.mean(v_plt)))

plt.plot(t_plt, c_plt)
plt.plot(t_plt, v_plt)
plt.show()

