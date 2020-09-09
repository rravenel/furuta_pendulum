import json
import matplotlib.pyplot as plt
import odrive
import time

print("\nPlot current and position.\n")

# Findings:
# Current oscillates w/ control loop, expected
# Ave. current varies w/ position - reflects torque ripple

# 0.3043A, -0.3038A are min to keep it spinning

cpr = 8192
step = 5
settle = 0.2
sample = 100

buf_f = []
buf_r = []

def datum(i, out):
		x.controller.pos_setpoint = i
		time.sleep(settle)
		p = x.encoder.pos_cpr

		buf = []
		for s in range(sample):
			buf.append(x.motor.current_control.Iq_measured)
		
		c = sum(buf) / sample

		out.append((p, c))
		

def measure():
	scale = 1	
	start = 0

	i = start
	print("\nForward...")
	while i < cpr/scale + start:
		print("%d" % (int(100 * i/(cpr/scale))))
		datum(i, buf_f)
		i += step

	print("\nReverse...")
	while i > start:
		print("%d" % (int(100 * i/(cpr/scale))))
		datum(i, buf_r)
		i -= step

print("Connecting...")
d = odrive.find_any()
print("Connected.")
x = d.axis0

x.controller.config.control_mode = 3

measure()

buf_f.sort(key=lambda tup: tup[0])
buf_r.sort(key=lambda tup: tup[0])

data = {"forward": buf_f, "reverse": buf_r}
with open("data/pos_cur_b.json", 'w') as file:
	json.dump(data, file)

print("Done")