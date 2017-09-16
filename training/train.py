#!/usr/bin/python

import csv
import numpy as np
from scipy.misc import imread
import cv2
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers import concatenate, Input
from keras.models import Model
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
import cPickle as pickle
import json
import sys

X_train = []
S_train = []
y_train = []

print("input is path to starting model, new model base filename, input pkl file to train on, num iterations")
print("./train.py ~/code/ARCRacing/models/office_set_predict_ahead_90_19.h5 september_track ~/code/ARCRacing/training/september_1.pkl 3")

starting_model = sys.argv[1]
base_output_model =  sys.argv[2]
#input_file = sys.argv[3]
num_iterations = int(sys.argv[3])


#we want to predict the steering angle for the next second of video. Here we 
#skip each 3 frames and predict on them
num_predict_ahead_frames_to_use = 90
predict_ahead_step_rate = 3
num_predict_ahead_frames = num_predict_ahead_frames_to_use//predict_ahead_step_rate



drives = ['V79_run_warehouse_1.pkl',"V79_run_warehouse_2.pkl"]
for drive in drives:
    with open('{}'.format(drive), 'rb') as f:
        # data = pickle.load(f,encoding='latin1') #if loading from python 3
        data = pickle.load(f)
        
    X, S, Y = (data['images'], 
        data['sensors'], 
        data['steering_throttle'].astype(np.float64))
    X_train.extend(X) #images
    S_train.extend(S) #sensors
    y_train.extend(Y) #steering
    
#     #flip left to right for augmented data
#     X, S, Y = (np.array([np.fliplr(x) for x in data['images']]),
#         data['sensors'],
#         np.negative(data['steering_throttle'].astype(np.float64)))
               
#     X_train.extend(X) #images
#     S_train.extend(S) #sensors
#     y_train.extend(Y) #steering
    
X_train = np.array(X_train)
S_train = np.array(S_train)
y_train = np.array(y_train)

print("Shape of inputs", X_train.shape, S_train.shape, y_train.shape)


def create_model():
    model = Sequential()
    
    #inputs
    image_input = Input(shape=(80, 320, 3), name='image_input', dtype='float32')
    sensor_input = Input(shape=(1,), name='sensor_input', dtype='float32')
    
    # preprocess
    X = Lambda(lambda x: x/255.0 - 0.5, name="lambda_1")(image_input)
    
    # conv1 layer
    X = Convolution2D(32, (5, 5), name="conv_1")(X)
    X = MaxPooling2D((2, 2), name="pool_1")(X)
    X = Activation('relu',name="relu_1")(X)
    
    # conv2 layer
    X = Convolution2D(64, (5, 5), name="conv_2")(X)
    X = MaxPooling2D((3, 3), name="pool_2")(X)
    X = Activation('relu', name="relu_2")(X)
    
    # conv3 layer
    X = Convolution2D(128, (3, 3), name="conv_3")(X)
    X = MaxPooling2D((2, 2), name="pool_3")(X)
    X = Activation('relu', name="relu_3")(X)

    # conv4 layer
    X = Convolution2D(128, (3, 3), name="conv_4")(X)
    X = MaxPooling2D((2, 2), name="pool_4")(X)
    X = Activation('relu', name="relu_4")(X)

    #add fully connected layers
    X = Flatten(name="flat_1")(X)
    
    #add in the speed, here we may add in other variables such 
    # as the last several throttle / speed/ steering angles, and other sensors
    X = concatenate([X, sensor_input], name="concate_1")
    
    # fc1
    X = Dense(1024, name="dnse_1")(X)
    X = Dropout(0.5, name="dropout_1")(X)
    X = Activation('relu', name="dense_relu_1")(X)
    
    # fc2
    X = Dense(128, name="dnse_2")(X)
    X = Dropout(0.5, name="dropout_2")(X)
    X = Activation('relu', name="dense_relu_2")(X)
    
    # fc2
    X = Dense(64, name="dnse_3")(X)
    X = Dropout(0.5, name="dropout_3")(X)
    X = Activation('relu', name="dense_relu_3")(X)
    
    #outputs are the next 10 frames
    steer_outputs = []
    for i in range(num_predict_ahead_frames):
        steer_output = Dense(1, name='steer_output_{}'.format(i))(X)
        steer_outputs.append(steer_output)
    
    

    #model = Model(inputs=[image_input, sensor_input], outputs=[steer_output, throttle_output])
    model = Model(inputs=[image_input, sensor_input], outputs=steer_outputs)

    loss_def = {"steer_output_{}".format(i) : "mse" for i in range(num_predict_ahead_frames)}
    loss_weight_def = {"steer_output_{}".format(i) : 1.0 for i in range(num_predict_ahead_frames)}
    
    # note, setting the loss weight to favor steering
    model.compile(optimizer='adam', loss=loss_def, loss_weights=loss_weight_def)

    return model


#create the model and save it as json
model = create_model()
model.load_weights("/home/nvidia/code/ARCRacing/models/{}".format(starting_model))  #<--last run


y_output = {"steer_output_{}".format(i) : y_train[:,i] for i in range(num_predict_ahead_frames)}

hist = []
for i in range(0,10):
    print("{} --------------".format(i))
    h = model.fit({'image_input': X_train, 'sensor_input': S_train[:,0]}, y_output, 
                  shuffle=True, epochs=10, validation_split=.3, batch_size=128)
    hist.append(h.history)
    with open('/home/nvidia/code/ARCRacing/models/history_{}.json'.format(base_output_model), 'w') as f:
            json.dump(hist, f)
            
    model.save("/home/nvidia/code/ARCRacing/models/{}_{}.h5".format(base_output_model, i))
    
print("done, go race")



