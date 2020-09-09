import odrive
import time


settle_default_s = 1
pre_interval_count = 100
pos_a = 2000
pos_b = 6000


cpr = 8192

def write(count, start, last, now, prev, pos):
	print("%d\t%f\t%f\t%f\t%f\t%f" % (count, now - start, now - last, pos, pos - prev, (pos - prev)/(now - last)))

def move_and_measure(pos):
	start = time.time()
	end = start + settle_default_s
	last = time.time()
	now = last
	p = 0
	prev = 0
	count = 0
	while now < end:
		last = now
		now = time.time()
		prev = p
		p = x.encoder.pos_cpr

		write(count, start, last, now, prev, p)
		
		count += 1
		if 100 == count:
			x.controller.pos_setpoint = pos

print("Measure settling time.")

d = odrive.find_any()

print("Connected to ODrive")

x = d.axis0

# save previous config
control_mode = x.controller.config.control_mode

# set to position control
x.controller.config.control_mode = 3

print("Setting initial position.")

# start pos
x.controller.pos_setpoint = pos_a
time.sleep(settle_default_s)

print("\nTest 1........\n")
move_and_measure(pos_b)

#print("\nTest 2........\n")
#move_and_measure(pos_a)

# restore previous config
x.controller.config.control_mode = control_mode

print("Done.")

