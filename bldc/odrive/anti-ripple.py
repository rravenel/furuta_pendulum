import json
import matplotlib.pyplot as plt
import odrive
import time

# Use position / current map to modify steady current
# See if velocity gets smoother

cpr = 8192
cur_min_f = 0.3043
cur_min_r = -0.3038
f_name = "data/pos_cur_b.json"
run_time_s = 30


data_in = None
map = [0] * cpr

def round(x):
	i = int(x)
	dec = x - i
	if dec >= 0.5:
		return i+1
	return i


def next_non_zero(start):
	for i in range(start+1, cpr):
		if 0 != map[i]:
			return i
	return -1

# generate complete position / holding-current map
# compies a given value to adjacent positions spanning half the distance to the next value
def interpolate():
	# build sparse map
	for d in f:
		p = d[0]
		c = d[1]

		p = round(p)
		map[p] = c

	# find first value
	first = next_non_zero(0)
	last = first
	next = next_non_zero(last)
	while next > 0:
		c_last = map[last]
		c_next = map[next]

		diff = next - last
		# 'next' pos already has a value, so don't count it
		diff -= 1

		if 1 == diff:
			map[last + 1] = c_next
		else:
			half = int(diff/2)

			for i in range(half):
				map[last + 1 + i] = c_last
	
			i = last + 1 + half
			while i < next:
				map[i] = c_next
				i += 1

		last = next
		next = next_non_zero(last)
		
	# fill in leading 0's
	c_leading = map[first]
	c_trailing = map[last]

	leading = first
	trailing = cpr - 1 - last
		
	diff = leading + trailing
	half = int(diff/2)

	for i in range(half):
		index = last + 1 + i
		if index > cpr - 1:
			index -= (cpr)
		map[index] = c_last

	i = last + 1 + half
	if i > cpr - 1:
		i -= (cpr)
	
	while i != first:
		map[i] = c_next
		i += 1
		if i > cpr - 1:
			i -= (cpr)

### load profile

with open(f_name, 'r') as file:
	data_in = json.load(file)

f = data_in["forward"]
interpolate()

### start motor

print("Connecting...")
drive = odrive.find_any()
print("Connected.")
x = drive.axis0

x.controller.config.control_mode = 1

buf_t = []
buf_p = []
buf_v = []

start = time.time()
now = start
while now - start < run_time_s:
	v = x.encoder.vel_estimate
	p = x.encoder.pos_cpr
	now = time.time()

	pr = round(p)
	if pr > cpr - 1:
		pr -= cpr
	c = map[pr]
	c += cur_min_f + 0.08 # arbitrary current high enough to keep it spinning
	x.controller.current_setpoint = c
	
	buf_t.append(now)
	buf_p.append(p)
	buf_v.append(v)

plt.plot(buf_t, buf_p)
plt.plot(buf_t, buf_v)
plt.show()

plt.plot(buf_p, buf_v)
plt.show()

