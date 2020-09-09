import matplotlib.pyplot as plt
import odrive
import time

print("Velocity control by position.")

cpr = 8192

def validation():
	now = time.time()
	t_start = now
	travel = 0

	while now < t_start + cpr / vel_target:
		now = time.time()
		t_diff = now - t_start

		pos_target = vel_target * t_diff
		if pos_target > cpr:
			pos_target -= cpr

		x.controller.pos_setpoint = pos_target

def loop2(buf_time, buf_vel):
	p_last = x.encoder.pos_cpr
	now = time.time()
	t_start = now
	t_last = t_start
	travel = 0

	while now < t_start + cpr / vel_target:
		p = x.encoder.pos_cpr
		now = time.time()
		t_loop = now - t_last
		t_last = now

		p_diff = p - p_last
		if p_diff < 1:
			p_diff += cpr
		p_last = p

		vel = p_diff / t_loop
	
		buf_time.append(now - t_start)
		buf_vel.append(int(vel))

		t_diff = now - t_start
		pos_target = vel_target * t_diff
		if pos_target > cpr:
			pos_target -= cpr

		x.controller.pos_setpoint = pos_target

	
def loop3(buf_time, buf_vel):
	p_last = x.encoder.pos_cpr
	p_start = p_last
	now = time.time()
	t_start = now
	t_last = t_start
	travel = 0

	while now < t_start + 5 * cpr / vel_target:
		p = x.encoder.pos_cpr
		now = time.time()
		t_loop = now - t_last
		t_last = now

		p_diff = p - p_last
		if p_diff < 1:
			p_diff += cpr
		p_last = p

		vel = p_diff / t_loop
	
		buf_time.append(now - t_start)
		buf_vel.append(p)

		t_diff = now - t_start
		pos_target = vel_target * t_diff + p_start
		if pos_target > cpr:
			pos_target -= cpr

		x.controller.pos_setpoint = pos_target


vel_target = 100000

d = odrive.find_any()
print("Connected.")
x = d.axis0

buf_pos = []
buf_time = []
loop3(buf_time, buf_pos)

#validation()

buf_vel = []
for i in range(min(len(buf_time),len(buf_pos))):
	if 0 == i:
		buf_vel.append(0)
	else:
		t_now = buf_time[i]
		t_last = buf_time[i-1]
		t_diff = t_now - t_last

		p_now = buf_pos[i]
		p_last = buf_pos[i-1]
		p_diff = p_now - p_last
		if p_diff < 1:
			if p_diff < (-1 * cpr/2):
				p_diff += cpr

		v = p_diff / t_diff

		#print("i: %d\td: %d\tt: %f\tv: %d" % (i, p_diff, 1000 * t_diff, int(v)))
		buf_vel.append(v)


#plt.plot(buf_time, buf_vel)
plt.plot(buf_pos, buf_vel)
plt.show()
