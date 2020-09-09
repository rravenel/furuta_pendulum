from __future__ import absolute_import, division, print_function, unicode_literals

import json
import numpy as np
import os
import random
import tensorflow as tf


TARGET = "target"
CURRENT = "i"
OBSERVATION = "obs"

HISTORY = 2

model_path = "current_network_2x3_b.h5"
data_name_prefix = "current_data_3_b_"
data_dir = "../data"

epoch_ = 50
#epoch_ = 20
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

def formatFloat(data):
    d = data[0]
    current = d[CURRENT]
    
    if 1 > len(current):
        print("No current data in first sample; exiting.")
        exit()

    datum = current[0]
    if isinstance(datum, float):
        print("is float")
        return True
    print("is not float")
    return False

def averageCurrent(current, floatType=False):
    buf = []
    for c in current:
        if not floatType:
            c = c[1]
        buf.append(c)
        
    return sum(buf)/float(len(buf))

def prepData(data):    
    for d in data:
        current = d[CURRENT]
        
        target = d[TARGET]
        obs = d[OBSERVATION]
        
        # target current, arm vel, pole pos, pole vel, average current
        p = [obs[0], obs[1], obs[2], obs[3], target, current]

        data2.append(p)
    
        
def split_2(data):
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

split_2(data2)

model = model_regression()

model.fit(train, train_output, validation_split = v_split, epochs=epoch_)

print(model.evaluate(test,  test_output, verbose=2))

model.save(model_path)

print("Saved model to: %s" % model_path)




