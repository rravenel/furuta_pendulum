from __future__ import absolute_import, division, print_function, unicode_literals

import json
import numpy as np
import os
import random
import tensorflow as tf
import time
import warnings

# numpy warnings because of tensorflow
warnings.filterwarnings("ignore", category=FutureWarning, module='tensorflow')

start = time.time()

ERR = "err"
OBS = "obs"

HISTORY = 2

model_path = "rnd_obs_policy_network_3x32-100e.h5"
data_name_prefix = "odsp_data_obs_"
data_dir = "../data"

epoch_ = 100
#epoch_ = 2
train_ = 0.8
test_ = 0.1
validate_ = 0.1
v_split = 0.222

## load data ##

data1= []
data2 = []

files = os.listdir(data_dir)
for name in files:
    if not name.startswith(data_name_prefix):
        continue
    with open(data_dir +'/' + name, 'r') as file:
	    data1 += json.load(file)
print("Data size: %d" % len(data1))
        
random.shuffle(data1)

## generate train/test/validate sets ##

train = []
train_output = []
validate = []
validate_output = []
test = []
test_output = []

def prepData2(data):
    for i in range(len(data)-1):
        d_prev = data[i]
        
        err_prev = d_prev[ERR]
        obs_prev = d_prev[OBS]

        d_cur = data[i+1]
        err_cur = d_cur[ERR]
        obs_cur = d_cur[OBS]
        
        prev_command = obs_cur[0] + err_cur
        obs_prev.pop() # remove target arm position
        
        data2.append([prev_command] + obs_prev + [err_cur])

def prepData(data):    
    for d in data:
        obs = d[OBS]
        err = d[ERR]

        obs.pop() # remove target arm position
        s = [obs[0] + err] + obs + [err]
        
        data2.append(s)
    
        
def split(data):
    for i in range(HISTORY, len(data)):
        buf = []
        for j in range(-(HISTORY), 1):
            d = data[i+j]
            buf += d
        output = buf.pop()
        
        r = random.random()
        if r <= train_ + test_:
            dest = train
            dest_output = train_output
        else:
            dest = test
            dest_output = test_output

        dest_output.append(output)
        dest.append(buf)

def model_regression():
	model = tf.keras.Sequential([
		tf.keras.layers.Dense(32, activation=tf.nn.softsign, input_shape=[17]),
		tf.keras.layers.Dense(32, activation=tf.nn.softsign),
		tf.keras.layers.Dense(32, activation=tf.nn.softsign),
		tf.keras.layers.Dense(1)
	])
	
	optimizer = tf.keras.optimizers.RMSprop(0.00001)
	
	#model.compile(loss='mean_absolute_error',
	model.compile(loss='mean_squared_error',
	              optimizer=optimizer,
	              metrics=['mean_absolute_error', 'mean_squared_error'])
	
	return model


## train and evaluate ##

prepData2(data1)

split(data2)

model = model_regression()

model.fit(train, train_output, validation_split = v_split, epochs=epoch_)

print(model.evaluate(test,  test_output, verbose=2))

model.save(model_path)

print("Saved model to: %s" % model_path)

print("Duration: %.3fm" % ((time.time() - start)/60.0))


