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
rev_settle = 50
rev_sample = 100
#rev_settle = 50
#rev_sample = 10
pos_pad = 500

buf_f = []
buf_r = []

def lap_forward(laps, record, buf):
	count = 0
	started = False
	while count < laps:
		while not started or (started and p > pos_pad):
			p = x.encoder.pos_cpr
			if record:
				buf	.append((time.time(), p))
			if p > 2 * pos_pad and p < cpr - pos_pad:
				started = True
		count += 1
		started = False
		print("Laps: %d/%d" % (count, laps))

def measure_forward():
	p_start = x.encoder.pos_cpr

	# get it spinning
	print("\nLaunch forward!")
	x.controller.current_setpoint = start_cur_f
	lap_forward(1, False, None)

	# let it settle
	print("Settling...")
	x.controller.current_setpoint = min_cur_f
	lap_forward(rev_settle, False, None)

	# record
	print("Recording...")
	x.controller.current_setpoint = min_cur_f
	lap_forward(rev_sample, True, buf_f)
	
	x.controller.current_setpoint = end_cur
	print("Complete forward.")



def lap_reverse(laps, record, buf):
	count = 0
	started = False
	while count < laps:
		while not started or (started and p < cpr - pos_pad):
			p = x.encoder.pos_cpr
			if record:
				buf	.append((time.time(), p))
			if p < cpr - 2 * pos_pad and p > pos_pad:
				started = True
		count += 1
		started = False
		print("Laps: %d/%d" % (count, laps))

def measure_reverse():
	p_start = x.encoder.pos_cpr

	# get it spinning
	print("\nLaunch reverse!")
	x.controller.current_setpoint = start_cur_r
	lap_reverse(1, False, None)

	# let it settle
	print("Settling...")
	x.controller.current_setpoint = min_cur_r
	lap_reverse(rev_settle, False, None)

	# record
	print("Recording...")
	x.controller.current_setpoint = min_cur_r
	lap_reverse(rev_sample, True, buf_r)
	
	x.controller.current_setpoint = end_cur
	print("Complete reverse.")


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
with open("data/const_cur_pos_3_full_raw.json", 'w') as file:
	json.dump(data, file)


print("Done")