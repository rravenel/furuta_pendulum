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
rev_settle = 30
rev_sample = 100
#rev_settle = 15
#rev_sample = 10
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

data = {"f": buf_f, "r": buf_r}
with open("data/const_cur_pos.json", 'w') as file:
	json.dump(data, file)


print("Done")