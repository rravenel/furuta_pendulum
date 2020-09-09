import odrive
import time

# measure bandwidth of current control - settling time
# ~10ms, 100Hz

# 0 -> 1.0: 0.008, 0.009, 0.005
# 0 -> 1.5: 0.005, 0.010, 0.008
# 0 -> -1.5: 0.004, 0.006, 0.006
# 0 -> 2.0: 0.006, 0.011, 0.013
# 0 -> 2.5: 0.008, 0.016, 0.009
# 0 -> 3.0: 0.011, 0.015, 0.013
# 0 -> 4.0: 0.011, 0.008, 0.006
# 0 -> 8.0: 0.011, 0.054, 0.006

c_set = 5
c_end = 0
t_sample = 0.05


print("Connecting...")
d = odrive.find_any()
print("Connected")
x = d.axis0

x.controller.config.control_mode = 1

def test(target):
	c = x.motor.current_control.Iq_measured
	now = time.time()
	print("\nStart - t: %f\tc: %f\n" % (now, c))
	
	v_last = 0
	t_start = 0
	now = t_start
	while now - t_start < t_sample:
		if 0 == t_start:
			x.controller.current_setpoint = target
			t_start = time.time()
	
		c = x.motor.current_control.Iq_measured
		now = time.time()
		print("t: %f\tc: %f" % (now, c))
	x.controller.current_setpoint = c_end
	time.sleep(3)

test(c_set)
test(c_set)
test(c_set)

x.controller.config.control_mode = 3
