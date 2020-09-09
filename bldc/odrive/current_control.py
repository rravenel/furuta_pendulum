import matplotlib.pyplot as plt
import odrive
import time

print("Experiments with current control.")


encoder_rpm_max = 5000
vel_limit = 136534.0
current_limit = 20
cpr = 8192

dc = 0.01
c_gain = 0.05
current_max = 2
vel_max = 10000
vel_target = 1000
vel_buf_scale = 1/15

def record(t, c, p):
	global p_last
	global t_last

	buf_t.append(t - t_start)
	buf_c.append(1000 * c)
	buf_p.append(p)

	p_diff = p - p_last
	if p_diff < (-1 * cpr/2):
		p_diff += cpr
	p_last = p

	t_diff = t - t_last
	t_last = t

	v = p_diff / t_diff
	buf_v.append(v * vel_buf_scale)

	print("mA: %d\tp: %f\tv: %f" % (int(1000 * c), p, v ))


print("Connecting...")
d = odrive.find_any()
print("Connected.")
x = d.axis0

x.controller.config.control_mode = 3
x.controller.pos_setpoint = 4000
time.sleep(1)
x.controller.config.control_mode = 1

buf_t = []
buf_c = []
buf_p = []
buf_v = []

p_start = x.encoder.pos_cpr
p_last = p_start
t_start = time.time()
t_last = t_start
now = t_start
current = 0.5
travel = 0

count = 0
while travel < cpr and now - t_start < 2*cpr / vel_target:
	count += 1
	p = x.encoder.pos_cpr
	now = time.time()
	
	dp = p - p_last
	if dp < (-1 * cpr/2):
		dp += cpr
	travel += dp
	#print("current pos: %f\tdp: %r\ttravel: %f" % (p, dp, travel))

	#p_target = (now - t_start) * vel_target + p_start
	#if p > p_target:
	#	current -= dc
	#if p < p_target:
	#	current += dc

	#if current < 0:
	#	current = 0
	#if current > current_max:
	#	current = current_max

	v = 0
	if len(buf_v) > 0:
		v = buf_v[-1] / vel_buf_scale

	#if v < vel_target:
	#	current += dc
	#if v > vel_target:
	#	current -= dc

	v_err = (vel_target - v) / vel_target
	current += v_err * c_gain

	#if current < 0:
	#	current = 0
	if current > current_max:
		current = current_max
	if current < -1 * current_max:
		current = -1 * current_max		

	if v > vel_limit or v < -1 * vel_limit:
		current = 0

	x.controller.current_setpoint = current
	record(now, current, p)



x.controller.config.control_mode = 3

print("count: %d\tlen: %d" % (count, len(buf_t)))
print("travel: %f\ttime: %f" % (travel, now - t_start))

plt.plot(buf_t, buf_p)
plt.plot(buf_t, buf_v)
plt.plot(buf_t, buf_c)
plt.show()