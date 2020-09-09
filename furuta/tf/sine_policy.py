from __future__ import absolute_import, division, print_function, unicode_literals

import json
import numpy as np
import os
import random
import tensorflow as tf
import time

start = time.time()

ERR = "err"
VEL = "vel"
OBS = "obs"

HISTORY = 2

model_path = "sine_policy_network_3x32.h5"
data_name_prefix = "odsp_data_"
data_dir = "../data"

epoch_ = 500
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

def prepData(data):    
    for d in data:
        obs = d[OBS]
        err = d[ERR]
        
        s = obs + [err]
        
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
		tf.keras.layers.Dense(32, activation=tf.nn.relu, input_shape=[17]),
		tf.keras.layers.Dense(32, activation=tf.nn.relu),
		tf.keras.layers.Dense(32, activation=tf.nn.relu),
		tf.keras.layers.Dense(1)
	])
	
	optimizer = tf.keras.optimizers.RMSprop(0.00001)
	
	model.compile(loss='mean_absolute_error',
	              optimizer=optimizer,
	              metrics=['mean_absolute_error', 'mean_squared_error'])
	
	return model


## train and evaluate ##

prepData(data1)

split(data2)

model = model_regression()

model.fit(train, train_output, validation_split = v_split, epochs=epoch_)

print(model.evaluate(test,  test_output, verbose=2))

model.save(model_path)

print("Saved model to: %s" % model_path)

print("Duration: %.3fm" % ((time.time() - start)/60.0))


