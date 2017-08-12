from __future__ import division, print_function
import numpy as np
import pickle
import json
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from datetime import datetime

# load some data
X_train = []
y_train = []

drives = ['small_data.pkl']
for drive in drives:
    with open(drive, 'rb') as f:
        data = pickle.load(f)
    X_train = data['X']
    y_train = data['y']


def create_model():
    model = Sequential()
    
    # preprocess
    model.add(Lambda(lambda x: x/255.0 - 0.5, input_shape=(80, 320, 3)))
    
    # conv1 layer
    model.add(Convolution2D(32, (5, 5)))
    model.add(MaxPooling2D((2, 2)))
    model.add(Activation('relu'))
    
    # conv2 layer
    model.add(Convolution2D(64, (5, 5)))
    model.add(MaxPooling2D((3, 3)))
    model.add(Activation('relu'))
    
    # conv3 layer
    model.add(Convolution2D(128, (3, 3)))
    model.add(MaxPooling2D((2, 2)))
    model.add(Activation('relu'))

    # conv4 layer
    model.add(Convolution2D(128, (3, 3)))
    model.add(MaxPooling2D((2, 2)))
    model.add(Activation('relu'))

    #add fully connected layers
    model.add(Flatten()) #Flatten input image
    
    # fc1
    model.add(Dense(1024))
    model.add(Dropout(0.5))
    model.add(Activation('relu'))
    
    # fc2
    model.add(Dense(128))
    model.add(Dropout(0.5))
    model.add(Activation('relu'))
    
    # fc2
    model.add(Dense(64))
    model.add(Dropout(0.5))
    model.add(Activation('relu'))
    
    model.add(Dense(1)) #output layer with 1 regression value
    model.compile(loss="mse", optimizer="adam")

    return model


model=create_model()
model.load_weights("model.h5")

print("starting to predict 1000 frames")
start = datetime.now()
for i in range(1000):
    print(np.array([X_train[i % 20]]).shape)
    predicted = model.predict(np.array([X_train[i % 20]]))[0]
end = datetime.now()
print("elapsed time: ", (end - start).total_seconds())
print("inference time per image: ", (end - start).total_seconds()/1000)
