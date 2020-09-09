import json
import matplotlib.pyplot as plt

f_name = "pos_cur.json"

data = None
with open(f_name, 'r') as file:
	data = json.load(file)

f = data["forward"]
r = data["reverse"]

print("len(f): %d\tlen(r): %d" % (len(f), len(r)))

p_f = []
c_f = []

for d in f:
	p_f.append(d[0])
	c_f.append(d[1])

plt.plot(p_f, c_f)
plt.show()