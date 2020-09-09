import odrive
import time

count = 1000

def measure():
	t = time.time()
	t2 = time.time()
	d = t2 - t
	
	print("Clock latency = %f" % (d))
	
	t3 = time.time()
	s = od.serial_number
	t4 = time.time()
	d2 = t4 - t3
	d3 = d2 - d
	f = 1/d3	

	#print("Serial number: %s\n" % (s))
	print("Measured round trip: %f" % (d2))
	print("Corrected round trip: %f" % (d3))
	print("Bandwidth: %fHz\n" % (f))

	return f


print("Measure ODrive USB latency")

od = odrive.find_any()

sum = 0
for i in range(count):
	sum += measure()

ave = sum/count

print("Average latency: %fHz" % (ave))

print("Done")