import json
import matplotlib.pyplot as plt
import numpy as np
import os


ERR = "err"
OBS = "obs"

data_name_prefix = "odsp_data_obs_"
data_dir = "../data"


data1= []
data2 = []

files = os.listdir(data_dir)
for name in files:
    if not name.startswith(data_name_prefix):
        continue
    with open(data_dir +'/' + name, 'r') as file:
	    data1 += json.load(file)
print("Data size: %d" % len(data1))

def prepData(data):    
    for d in data:
        obs = d[OBS]
        err = d[ERR]

        obs.pop() # remove target arm position
        s = obs + [err]
        
        data2.append(s)


def plotData(data):
    buf_index = []
    buf_pos = []
    buf_err = []
    buf_cmd = []
    
    size = 100
    for i in range(size):
        d = data[i]
        
        buf_index.append(i)
        buf_pos.append(d[0])
        buf_err.append(5*d[4])
        buf_cmd.append(d[0] + d[4])
    
    buf_index.append(size)
    buf_pos.insert(0, 0)    
    buf_err.insert(0, 0)    
    buf_cmd.append(0)
    
    plt.plot(buf_index, buf_pos)
    plt.plot(buf_index, buf_cmd)
    plt.plot(buf_index, buf_err)

prepData(data1)
plotData(data2)

plt.show()