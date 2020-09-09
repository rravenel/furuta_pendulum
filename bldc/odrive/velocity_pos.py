import json
import matplotlib.pyplot as plt
import odrive
import time

print("\nPlot velocity by position.\n")

cpr = 8192
min_cur_f = 0.3043
min_cur_r = -0.3038
start_cur_f = 1.2
start_cur_r = -1.2
end_cur = 0.0

rev_launch = 1
#rev_settle = 30
#rev_sample = 50
rev_settle = 15
rev_sample = 10
pos_pad = 500

buf_f = []
buf_r = []


def lap_reverse(record, buf):
	p = 0
	started = False
	t_start = time.time()
	while not started or (started and p < cpr - pos_pad):
		p = x.encoder.pos_cpr
		if record:
			buf	.append((time.time() - t_start, p))
		if p < cpr - 2 * pos_pad and p > pos_pad:
			started = True

def measure_reverse():
	laps = 0
	p_start = x.encoder.pos_cpr

	# get it spinning
	print("\nLaunch reverse!")
	x.controller.current_setpoint = start_cur_r
	lap_reverse(False, None)

	# let it settle
	print("Settling...")
	x.controller.current_setpoint = min_cur_r
	for i in range(rev_settle):
		lap_reverse(False, None)
		laps += 1
		print("Laps: %d/%d" % (laps, rev_settle))

	# record
	print("Recording...")
	x.controller.current_setpoint = min_cur_r
	for i in range(rev_sample):
		buf = []
		lap_reverse(True, buf)

		#print("Lap len: %d" % (len(buf)))

		buf_r.append(buf)
		laps += 1
		print("Laps: %d/%d" % (laps - rev_settle, rev_sample))
	
	x.controller.current_setpoint = end_cur
	print("Complete reverse.")


def lap_forward(record, buf):
	p = 0
	started = False
	t_start = time.time()
	while not started or (started and p > pos_pad):
		p = x.encoder.pos_cpr
		if record:
			buf	.append((time.time() - t_start, p))
		if p > 2 * pos_pad and p < cpr - pos_pad:
			started = True

def measure_forward():
	laps = 0
	p_start = x.encoder.pos_cpr

	# get it spinning
	print("\nLaunch forward!")
	x.controller.current_setpoint = start_cur_f
	lap_forward(False, None)

	# let it settle
	print("Settling...")
	x.controller.current_setpoint = min_cur_f
	for i in range(rev_settle):
		lap_forward(False, None)
		laps += 1
		print("Laps: %d/%d" % (laps, rev_settle))

	# record
	print("Recording...")
	x.controller.current_setpoint = min_cur_f
	for i in range(rev_sample):
		buf = []
		lap_forward(True, buf)
		buf_f.append(buf)
		laps += 1
		print("Laps: %d/%d" % (laps - rev_settle, rev_sample))
	
	x.controller.current_setpoint = end_cur
	print("Complete forward.")


print("Connecting...")
d = odrive.find_any()
print("Connected.")
x = d.axis0

x.controller.config.control_mode = 3
x.controller.pos_setpoint = 0
x.controller.config.control_mode = 1
measure_forward()

x.controller.config.control_mode = 3
x.controller.pos_setpoint = 0
x.controller.config.control_mode = 1
measure_reverse()

x.controller.config.control_mode = 3

####### compute velocity
v_filter_1 = 1000000
v_filter_2 = 100000

# forward
filter_1_count = 0
buf_v_f = []
for lap in buf_f:
	spike = False
	buf_v = []
	t_last = lap[0][0]
	p_last = lap[0][1]
	for i in range(1, len(lap)):
		t = lap[i][0]
		p = lap[i][1]

		dp = p - p_last
		dt = t - t_last

		p_last = p
		t_last = t

		v = dp/dt
		if v > -1 * v_filter_1:
			buf_v.append((t, v))
			if v > v_filter_2:
				spike = True
		else:
			filter_1_count += 1

	if not spike:
		buf_v_f.append(buf_v)

print("forward filter_1_count: %d" % (filter_1_count))

buf_t_f = []
buf_v2_f = []
for lap in buf_v_f:
	for d in lap:
		buf_t_f.append(d[0])
		buf_v2_f.append(d[1])

#plt.plot(buf_t_f, buf_v2_f)
#plt.show()


# reverse
filter_1_count = 0
buf_v_r = []
for lap in buf_r:
	spike = False
	buf_v = []
	t_last = lap[0][0]
	p_last = lap[0][1]
	for i in range(1, len(lap)):
		t = lap[i][0]
		p = lap[i][1]

		dp = p - p_last
		dt = t - t_last

		p_last = p
		t_last = t

		v = dp/dt
		if v < v_filter_1:
			buf_v.append((t, v))
			if v < -1 * v_filter_2:
				spike = True
		else:
			filter_1_count += 1

	if not spike:
		buf_v_r.append(buf_v)

print("reverse filter_1_count: %d" % (filter_1_count))
print("buf_v_r len: %d" % (len(buf_v_r)))

buf_t_r = []
buf_v2_r = []
for lap in buf_v_r:
	for d in lap:
		buf_t_r.append(d[0])
		buf_v2_r.append(d[1])

#plt.plot(buf_t_r, buf_v2_r)
#plt.show()

########### compute acceleration

# forward
buf_a_f = []
for lap in buf_v_f:
	buf_a = []
	t_last = lap[0][0]
	v_last = lap[0][1]
	for i in range(1, len(lap)):
		t = lap[i][0]
		v = lap[i][1]

		dv = v - v_last
		dt = t - t_last

		v_last = v
		t_last = t

		a = dv/dt
		buf_a.append((t, a))

	buf_a_f.append(buf_a)

buf_t_f = []
buf_a2_f = []
for lap in buf_a_f:
	for d in lap:
		buf_t_f.append(d[0])
		buf_a2_f.append(d[1])

plt.plot(buf_t_f, buf_a2_f)
plt.show()	

# reverse
buf_a_r = []
for lap in buf_v_r:
	buf_a = []
	t_last = lap[0][0]
	v_last = lap[0][1]
	for i in range(1, len(lap)):
		t = lap[i][0]
		v = lap[i][1]

		dv = v - v_last
		dt = t - t_last

		v_last = v
		t_last = t

		a = dv/dt
		buf_a.append((t, a))

	buf_a_r.append(buf_a)

buf_t_r = []
buf_a2_r = []
for lap in buf_a_r:
	for d in lap:
		buf_t_r.append(d[0])
		buf_a2_r.append(d[1])

plt.plot(buf_t_r, buf_a2_r)
plt.show()	

######### plot position/time

# forward data
buf_t_f = []
buf_p_f = []
for lap in buf_f:
	for d in lap:
		buf_t_f.append(d[0])
		buf_p_f.append(d[1])

#plt.plot(buf_t_f, buf_p_f)
#plt.show()

# reverse data
buf_t_r = []
buf_p_r = []
for lap in buf_r:
	for d in lap:
		buf_t_r.append(d[0])
		buf_p_r.append(d[1])

#plt.plot(buf_t_r, buf_p_r)
#plt.show()

###############################
ripple_raw = []
ripple_raw.append((buf_t_f[0], buf_p_f[0]))	
ripple_raw.append((buf_t_f[1], buf_p_f[1], buf_v_f[0]))
for i in range(2, len(buf_t_f)):
	ripple_raw.append((buf_t_f[i], buf_p_f[i], buf_v_f[i-1], buf_a_f[i-2]))



print("Done")