import json
import matplotlib.pyplot as plt

'''
Visualize current vs position data
'''


file_name = "const_cur_pos_3_raw.json"
cpr = 8192




data = None

with open(file_name, 'r') as file:
	data = json.load(file)

f = data["f"]
r = data["r"]


# forward

##### time align revolutions

# 1) find rev end indexes
rev_end_index = []
p_last = 0
for i in range(len(f)):
	d = f[i]
	p = d[1]
	if 0 == i:
		p_last = p
		continue

	dp = p - p_last
	p_last = p
	if dp < 1:
		rev_end_index.append(i-1)
		rev_count = len(rev_end_index)
		rev_points = 0
		if rev_count < 2:
			rev_points = rev_end_index[0]
		else:
			rev_points = rev_end_index[rev_count - 1] - rev_end_index[rev_count - 2]
		print("Rev points: %d" % (rev_points))

# 2) find rev intervals
t_last = f[0][0]
rev_interval = []
for i in rev_end_index:
	t = f[i][0]
	interval = t - t_last
	t_last = t
	rev_interval.append(interval)
	print("Interval: %f" % (interval))

# 3) average rev interval
rev_interval_ave = sum(rev_interval) / len(rev_interval)
print("Interval average: %f" % (rev_interval_ave))

# 4) scale intervals to fit average
rev_start_index = 0
t_rev_start = f[0][0]
for i in rev_end_index:
	t_end = f[i][0]
	t_diff = t_end - t_rev_start
	t_factor = t_diff / rev_interval_ave
	
	for j in range(rev_start_index, i+1):
		t = f[j][0]
		t_diff = t - t_rev_start
		t_new = t_diff * t_factor
		t_set = t_new + t_rev_start
		f[j][0] = t_set
		

	rev_start_index = i + 1
	t_rev_start = f[rev_start_index][0]

#####################

t_last = None
p_last = None
v_last = None

t_diff_sum = 0
t_diff_count = 0

smooth = 1
for i in range(smooth, len(f)):
	d_last = f[i - smooth]	
	t_last = d_last[0]
	p_last = d_last[1]	

	d = f[i]
	t = d[0]
	p = d[1]

	dt = t - t_last
	t_diff_sum += dt
	t_diff_count += 1

	dp = p - p_last
	if dp < 0:
		dp += cpr

	v = dp/dt
	if v < 1:
		continue

	d.append(v)

	if len(d_last) < 3:
		continue
	v_last = d_last[2]
	
	dv = v - v_last

	a = dv/dt

	d.append(a)

print("Sample count: %d\tAve. sample interval: %f" % (t_diff_count, t_diff_sum/t_diff_count))
	
f.sort(key=lambda tup: tup[1])

buf_t = []
buf_p = []
buf_v = []
buf_v2 = []
buf_a = []

def plot_prep_simple():
	for d in f:
		if len(d) < 4:
			continue
		buf_t.append(d[0])
		buf_p.append(d[1])
		buf_v.append(d[2])
		buf_a.append(d[3])
		#if d[1] > 8000:
		#	break

def plot_prep_bucket():
	size = 128
	bucket = 0
	v_sum = 0
	count = 0

	for d in f:
		if len(d) < 3:
			continue

		p = d[1]
		if p > bucket * size + size:
			if count > 0:
				v = v_sum / count
				buf_p.append(bucket*size + size/2)
				buf_v.append(v)

			v_sum = 0
			count = 0
			bucket += 1

		v_sum += d[2]
		count += 1

	

def plot_prep_smooth():	
	global buf_t
	global buf_p
	global buf_v 
	global buf_a 
	smooth2 = 1
	for i in range(smooth2, len(f)):
		missing_data = False
		v_sum = 0
		for j in range(smooth2):
			d = f[i - j]
			if len(d) < 3:
				missing_data = True
				break
			v_sum += d[2]
		if missing_data:
			continue
		v = v_sum / smooth2
	
		d = f[i]
		buf_t.append(d[0])
		buf_p.append(d[1])
		buf_v.append(v)
		#if d[1] > 8000:
		#	break
	
	for i in range(smooth2):
		buf_v2.append(buf_v[i])
	
	for i in range(smooth2, len(buf_v)):
		v_sum = 0
		for j in range(smooth2):
			v = buf_v[i - j]
			v_sum += v
		v = v_sum / smooth2
		buf_v2.append(v)
	buf_v = buf_v2
	buf_v = buf_v[20:]
	buf_p = buf_p[20:]
	buf_t = buf_t[20:]

	buf_a.append(0)
	for i in range(1, len(buf_v)):
		t = buf_t[i]
		t_last = buf_t[i-1]
		dt = t - t_last		

		v = buf_v[i]
		v_last = buf_v[i-1]
		dv = v - v_last

		a = dv/dt
		buf_a.append(a/10)
	buf_a[0] = buf_a[1]

	buf_a2 = []
	for i in range(smooth2):
		buf_a2.append(buf_a[i])
	for i in range(smooth2, len(buf_a)):
		a_sum = 0
		for j in range(smooth2):
			a = buf_a[i - j]
			a_sum += a
		a = a_sum / smooth2
		buf_a2.append(a)
	buf_a = buf_a2

plot_prep_bucket()

#plt.plot(buf_t, buf_a)
#plt.plot(buf_t, buf_v)
#plt.plot(buf_t, buf_p)
plt.plot(buf_p, buf_v)
#plt.plot(buf_p, buf_a)
plt.show()

