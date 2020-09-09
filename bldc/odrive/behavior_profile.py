import json
import odrive
import random
import time

'''
Run the motor randomly to collect behavior data.
'''
t_run = 2200
#t_run = 5
c_max = 10
c_safe = 1

# safe margin from 600000 limit
v_max = 400000
t_power = 0.01

count_sample_max = 4


def v_check(v):
	if v > v_max or v < (-1 * v_max):
		print("Overspeed!!!")
		return False
	return True

print("Connecting...")
d = odrive.find_any()
print("Connected")
x = d.axis0

x.controller.config.control_mode = 1
x.controller.current_setpoint = 0

buf_t = []
buf_c_set = []
buf_c_mes = []
buf_p = []
buf_v = []

buf_data = []

t_start = time.time()
t_now = t_start
c_last = 0
count_sample = 0
while t_now - t_start < t_run:
	v = x.encoder.vel_estimate
	if not v_check(v):
		c_next = 0
		count_sample = 0
	else:
		switch = random.randint(0,1)
		if switch == 0 and count_sample < count_sample_max:
			c_next = c_last
			count_sample += 1
		else:
			c_next = random.randint(-1 *1000 * c_max, 1000 * c_max)/1000
			if random.random() < 0.7:
				c_next = c_next / 7.5
			count_sample = 0
	if c_last != c_next:
		x.controller.current_setpoint = c_next
		c_last = c_next

	p = x.encoder.pos_cpr
	c = x.motor.current_control.Iq_measured

	t_now = time.time()

	buf_data.append((t_now, p, v, c, c_last))

for d in buf_data:
	print("t: %f\tp: %f\tv: %f\tc_sample: %f\tc_set: %f" % (d[0], d[1], d[2], d[3], d[4]))

with open("data/behavior.json", 'w') as file:
	json.dump(buf_data, file)