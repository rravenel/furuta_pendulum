import odrive
import time

'''
Calculate motor's angular moment of inertia

c: -2.831128	a: 2026	T: -0.023413	J: -0.000012
rene@LBB:~/work/odrive$ python3 acceleration_profiler.py 
Connecting...
Connected
c: -2.632490	a: 2025	T: -0.021771	J: -0.000011
rene@LBB:~/work/odrive$ python3 acceleration_profiler.py 
Connecting...
Connected
c: -1.916583	a: 1217	T: -0.015850	J: -0.000013
rene@LBB:~/work/odrive$ python3 acceleration_profiler.py 
Connecting...
Connected
c: 1.974330	a: -1205	T: 0.016328	J: -0.000014
rene@LBB:~/work/odrive$ python3 acceleration_profiler.py 
Connecting...
Connected
c: 1.755427	a: -1218	T: 0.014517	J: -0.000012
rene@LBB:~/work/odrive$ python3 acceleration_profiler.py 
Connecting...
Connected
c: -0.808336	a: 471	T: -0.006685	J: -0.000014
rene@LBB:~/work/odrive$ python3 acceleration_profiler.py 
Connecting...
Connected
c: -3.161901	a: 2434	T: -0.026149	J: -0.000011

'''


# measure accelleration over short, high current intervals
# stock rotor angular moment of inertia: 0.000012 Kg m2
# @1.0A, T = .007 Nm
# @2.0A, T = .016 Nm
# @3.0A, T = .023 Nm
# @3.5A, T = .026 Nm

c_set = 3.5
c_end = 0
t_sample = 0.2


print("Connecting...")
d = odrive.find_any()
print("Connected")
x = d.axis0

x.controller.config.control_mode = 1

def test(target):
	now = time.time()
	
	v_last = 0
	t_start = 0
	t_last = 0
	now = t_start
	while now - t_start < t_sample:
		if 0 == t_start:
			x.controller.current_setpoint = target
			t_start = time.time()
			t_last = t_start

		# let current stabilize
		time.sleep(0.01)

		v = x.encoder.vel_estimate	
		c = x.motor.current_control.Iq_measured
		now = time.time()
		dv = v - v_last
		dt = now - t_last
		a = dv/dt
	
		buf_c.append(c)
		buf_a.append(a)

		v_last = v
		t_last = now

	x.controller.current_setpoint = c_end
	time.sleep(3)

buf_c = []
buf_a = []

test(c_set)
test(c_set)
test(c_set)

c_ave = sum(buf_c) / len(buf_c)
a_ave = sum(buf_a) / len(buf_a)
a_rad = a_ave / 1303.8

T = 8.27 * c_ave / 1000
J = T / a_rad

print("c: %f\ta: %d\tT: %f\tJ: %f" % (c_ave, int(a_rad), T, J))
