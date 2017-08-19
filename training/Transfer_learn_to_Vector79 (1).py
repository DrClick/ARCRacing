
# coding: utf-8

# In[1]:

import csv
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns
from scipy.misc import imread, imsave
import cv2
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
import pickle
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
import json


# In[2]:

get_ipython().magic('matplotlib inline')
sns.set_context("poster")


# In[3]:

X_train = []
y_train = []

drives = ['vector79_run_1.pkl']
for drive in drives:
    with open(drive, 'rb') as f:
        data = pickle.load(f)
    X_train.extend(data['images'])
    y_train.extend(data['steering_throttle'].astype(np.float64))
    X_train.extend(np.array([np.fliplr(x) for x in data['images']]))
    y_train.extend(np.negative(data['steering_throttle'].astype(np.float64)))
    
X_train = np.array(X_train)
y_train = np.array(y_train)[:,[0]]


# In[4]:

print(X_train.shape, y_train.shape)


# In[5]:

#view some images
plt.imshow(X_train[100])


# In[25]:

# now we need to evenly distribute the steering angles
# its likely there is an over bias towards straight
sns.distplot(y_train)


# In[7]:

steering_bins = [x/10.0 for x in range(-10,10,1)]
steering_buckets = [[] for _ in steering_bins]
for i, y in enumerate(y_train):
    index = 0
    while index < len(steering_bins):
        if y < steering_bins[index]:
            steering_buckets[index].append(i)
            break
        index += 1
    if index  == len(steering_bins):
        steering_buckets[index-1].append(i)

print(steering_bins)
counts_buckets = [len(x) for x in steering_buckets]
print(counts_buckets)


# In[8]:

min_bucket = min(counts_buckets)
#create an even distribution from the buckets
distributed_indicies = []
for bucket in steering_buckets:
    distributed_indicies.extend(np.random.choice(bucket, min(len(bucket), min_bucket * 10), replace=False))
y_dist = y_train[distributed_indicies]
X_dist = X_train[distributed_indicies]
sns.distplot(y_dist)
print(len(y_dist))


# In[28]:

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


# In[29]:

# #create the model and save it as json
# model = create_model()
# with open("model.json", "w") as f:
#     json.dump(model.to_json(), f)


# In[6]:

#loading the model
from keras.models import model_from_json
with open("model.json") as f:
    model = model_from_json(json.load(f))
    model.compile(loss="mse", optimizer="adam")
model.load_weights("model.h5")


# In[34]:

history = []
for i in range(30):

    h = model.fit(X_train, y_train, shuffle=True, epochs=3, validation_split=.2, batch_size=64)
    history.append(h)
    model.save("data_model_tranfered_{}.h5".format(i))
    


# In[19]:

#create small test set to transfer to pi for testing
small_data = {
    "X": X_train[1500:1520].tolist(),
    "y": y_train[1500:1520].tolist()
}


# In[20]:

with open('small_data.json', 'w') as f:
    json.dump(small_data, f)


# In[22]:

#inference
img = X_train[1550]
actual = y_train[1550]
print(actual)
plt.imshow(img)


# In[23]:

model.predict(np.array([img]))


# In[24]:

0.04938826 * 45


# In[ ]:



