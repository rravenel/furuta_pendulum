import odrive
import time

# measure current oscillation frequency
# test current 0.0A
# Sample count: 4723		duration: 10
# Interval count: 2392	max: 0.027539	min: 0.001360
# period: 0.004174	freq: 239.605884
# c = 0.2A, f=186Hz
# c = -0.2A, f=167Hz
# c = 0.3A, f=148Hz

t_sample = 10
c_set = 0.3

print("Connecting...")
d = odrive.find_any()
print("Connected.")
x = d.axis0

print("Sample...")
x.controller.config.control_mode = 1
x.controller.current_setpoint = c_set

t_start = time.time()
now = t_start

buf = []
while now - t_start < t_sample:
	c = x.motor.current_control.Iq_measured
	now = time.time()
	buf.append((now, c))

x.controller.config.control_mode = 3

print("Sample count: %d" % (len(buf)))

print("Compute:")
interval = []
cross = []
c_last = buf[0][1] - c_set
for d in buf:
	c = d[1] - c_set
	if (c > 0) != (c_last > 0):
		t = d[0]
		cross.append(t)
		if 1 == len(cross):
			continue
		t_diff = t - cross[-2]
		if t_diff != 0:
			interval.append(t_diff)
	c_last = c

print("Interval count: %d\tmax: %f\tmin: %f" % (len(interval), max(interval), min(interval)))

period = sum(interval) / len(interval)
freq = 1/period

print("period: %f\tfreq: %f" % (period, freq))

print("Done")