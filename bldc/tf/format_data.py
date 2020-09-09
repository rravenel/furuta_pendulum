import json

cpr= 8192
vel_max = 600000
cur_max = 10.000001
t_max = 1000
J = .000012

data_in = None
with open("../odrive/data/behavior.json", 'r') as file:
	data_in = json.load(file)

ks = data_in[:1000]

data_out = []


def calc(data):
	for i in range(1, len(data)):
		d = data[i]

		buf = []

#		if d[1] == cpr:
#			d[1] = 0

		buf.append(d[1]/cpr)
		buf.append((1 + d[2]/vel_max)/2)
		
		d0 = data[i-1]
		dt = d[0] - d0[0]
		dv = d[2] - d0[2]
		a = dv/dt
		T = a*J

		buf.append((1 + T/t_max)/2)
		buf.append((1 + d[4]/cur_max * -1)/2)

		for b in buf:
			if b < 0 or b >= 1:
				print(buf)

		data_out.append(buf)

calc(data_in)
#calc(ks)

with open("data/dataset.json", 'w') as file:
	json.dump(data_out, file)

#for d in data_out:
#	print(d)