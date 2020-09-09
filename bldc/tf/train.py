from __future__ import absolute_import, division, print_function, unicode_literals

import json
import numpy as np
import random
import tensorflow as tf

#model_path = "data/model1.h5"
model_path = "data/model_tmp.h5"
data_path = "../odrive/data/behavior2.json"

#epoch_ = 200
epoch_ = 20
train_ = 0.8
test_ = 0.1
validate_ = 0.1
v_split = 0.222

## load data ##

data = None
with open(data_path, 'r') as file:
	data = json.load(file)

random.shuffle(data)

## generate train/test/validate sets ##

train = []
train_output = []
validate = []
validate_output = []
test = []
test_output = []

def split_1():
	for d in data:
		r = random.random()
		if r <= train_:
			train_output.append(d[-1])
			d.pop()
			train.append(d)
		elif r > train_ and r <= train_ + test_:
			test_output.append(d[-1])
			d.pop()
			test.append(d)
		else:
			validate_output.append(d[-1])
			d.pop()
			validate.append(d)

def split_2():
	for d in data:
		r = random.random()
		if r <= train_ + test_:
			train_output.append(d[-1])
			d.pop()
			train.append(d)
		else:
			test_output.append(d[-1])
			d.pop()
			test.append(d)

#for i in range(len(test)):	
#	print(test[i])
#	print(test_output[i])

## build network ##

def model_classifier():
	model = tf.keras.models.Sequential([
		tf.keras.layers.Flatten(input_shape=(3,)),
		tf.keras.layers.Dense(32, activation=tf.nn.relu),
		#tf.keras.layers.Dense(32, activation=tf.nn.softmax),
		tf.keras.layers.Dropout(0.2),
		tf.keras.layers.Dense(1, activation=tf.nn.softmax)
	])
	
	model.compile(optimizer='adam',
	              loss='sparse_categorical_crossentropy',
	              metrics=['accuracy'])

	return model

def model_regression():
	model = tf.keras.Sequential([
		tf.keras.layers.Dense(32, activation=tf.nn.relu, input_shape=[2]),
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

split_2()

model = model_regression()

model.fit(train, train_output, validation_split = v_split, epochs=epoch_)

print(model.evaluate(test,  test_output, verbose=2))

model.save(model_path)



#model2 = tf.keras.models.load_model(model_path)
#print(model2.evaluate(test,  test_output, verbose=2))

#sample = np.array([train[0]])
#expect = train_output[0]

#print(sample)
#print(expect)

#predictions = model2.predict(sample)
#print(predictions)



