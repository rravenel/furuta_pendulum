import odrive
import matplotlib.pyplot as plt
import time

'''
Measure internal drag/friction.  Spin it up and clock it slowing down.

1465rpm / 154 rad/s / 24.4Hz: -356 rad/s^2
2197rpm / 230 rad/s / 36.6Hz: -378 rad/s^2
2930rpm / 307 rad/s / 48.8Hz: -342 rad/s^2
3663rpm / 383 rad/s / 61.0Hz: -324 rad/s^2

'''
# max is 600,000
v_target = 500000
t_sample = 5
still_count = 20
c2rad = 1303.8
t_cut = 1.05
v_sample = 25

print("Connecting...")
d = odrive.find_any()
print("Connected")
x = d.axis0

x.controller.config.control_mode = 1
x.controller.current_setpoint = 0

def test():
	x.controller.config.control_mode = 2
	x.controller.vel_setpoint = v_target
	time.sleep(0.5)

	x.controller.config.control_mode = 1
	x.controller.current_setpoint = 0

	v_last = 0
	t_start = time.time()
	t_last = t_start
	now = t_start
	zero_v_count = 0
	while now - t_start < t_sample:
		if zero_v_count >= still_count:
			break

		v = x.encoder.vel_estimate
		v = v / c2rad
		now = time.time()
		dv = v - v_last
		dt = now - t_last
		a = dv/dt

		buf_t.append(now)	
		buf_v.append(v)
		buf_a.append(a)

		v_last = v
		t_last = now

		if 0 == int(v):
			zero_v_count += 1
		else:
			zero_v_count = 0


buf_t = []
buf_v = []
buf_a = []

count = 1
for i in range(count):
	test()

# throw out first sample from v = 0
buf_t = buf_t[1:]
buf_v = buf_v[1:]
buf_a = buf_a[1:]

data = []

drag_map_v = []
drag_map = []
buf_seg = []

t_start = buf_t[0]
for i in range(len(buf_t)):
	t = buf_t[i] - t_start
	v = buf_v[i]
	a = int(buf_a[i])

	print("#%d:\tt: %fs\tv: %frad/s\ta: %drad/s2" % (i, t, v, a))

	if t > 0.05 and t < t_cut:
		data.append(a)

	buf_seg.append(a)
	if i > 0 and 0 == i%v_sample:
		v_diff = buf_v[i-10] - v
		drag_map_v.append(v + v_diff/2)
		drag_map.append(sum(buf_seg)/len(buf_seg))
		buf_seg = []
		print("\tv: %f\td: %f" % (drag_map_v[-1], drag_map[-1]))

	# alter for rendering
	buf_t[i] = t
	buf_v[i] = 25 * v

drag = sum(data) / len(data)
print("Acceleration due to drag: %frad/s2" % (drag))

#plt.plot(buf_t, len(buf_t) * [0])
#plt.plot(buf_t, buf_a)
#plt.plot(buf_t, buf_v)
plt.plot(drag_map_v, len(drag_map) * [0])
plt.plot(drag_map_v, drag_map)
plt.show()
