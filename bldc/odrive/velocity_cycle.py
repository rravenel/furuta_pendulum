import odrive
import time

'''
Ramp velicity up and then back down.
'''

CTRL_MODE_VELOCITY_CONTROL = 2

cpr = 8192
interval = 0.5
max_v = 10
hold = 5

print("Velocity cycle: start.")

d = odrive.find_any()

print("Connected to Odrive")

x = d.axis0

x.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

print("Begin acceleration...")

for i in range(max_v):
	x.controller.vel_setpoint = i * cpr
	print("vel_setpoint = %d.  " % (i * cpr))
	time.sleep(interval)

print("Hold max_v for %d seconds..." % (hold))

time.sleep(hold)

print("Begin deceleration...")

for i in range(2, max_v):
	x.controller.vel_setpoint = (max_v - i) * cpr
	print("vel_setpoint = %d.  " % ((max_v - i) * cpr))
	time.sleep(interval)
x.controller.vel_setpoint = 0

print("Done")