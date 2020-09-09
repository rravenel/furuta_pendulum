import odrive
import time

print("See how fast we can get position data from ODrive via USB")

test_duration_s = 5
target_velocity = 8000

cpr = 8192
d = odrive.find_any()

print("Connected to ODrive")

x = d.axis0

# save previous config
control_mode = x.controller.config.control_mode

# set to velocity control
x.controller.config.control_mode = 2

x.controller.vel_setpoint = target_velocity

p_buf = []
t_buf = []

last_p = x.encoder.pos_cpr
now = time.time()
last_t = now
end = now + test_duration_s
while now < end:
	p = x.encoder.pos_cpr
	delta = p - last_p

	if delta < 1:
		last_p = last_p - cpr
		delta = p - last_p

	p_buf.append(delta)
	last_p = p

	now = time.time()
	t_buf.append(now - last_t)
	last_t = now


print("Sample count: %d" % (len(p_buf)))
print("Total rotation: %d" % (sum(p_buf)))
print("Average delta: %f" % (sum(p_buf) / len(p_buf)))
print("Average interval: %f" % (sum(t_buf) / len(t_buf)))

x.controller.vel_setpoint = 0

# restore previous config
x.controller.config.control_mode = control_mode

print("Done.")