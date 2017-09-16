#!/usr/bin/python

import numpy as np
from scipy.misc import imread
import pickle
import cv2
import sys
import csv

folder_to_process = sys.argv[1]
print("Processing ", folder_to_process)


def extract_data(folder):
    S = [] #image sequence id
    P = [] #shadow predictions 
    X = [] #images
    XX = [] #senor data (aka speed in this case)
    y = [] #actual steering angles (throttle in the future)
    
    with open('{}/_data.csv'.format(folder_to_process)) as f:
        reader = csv.reader(f)
        print(''.join(next(reader)))
        for line in reader:
            line = ''.join(line).split('|')
            image_seq_id= int(line[0])
    
            img = imread('{}/{}.jpg'.format(folder_to_process, image_seq_id))
            
            S.append(image_seq_id)
            X.append(img)
            #index 8 - throttle is -100 to 100
            #index 7 - rpm is 5000ish max
            XX.append([float(line[7])/5000, float(line[8])/100]) 
            y.append([float(line[5])/45.0]) # steering is -45 to 45

    return np.array(S), np.array(X), np.array(XX), np.array(y)


S_train, X_train, XX_train, y_train = extract_data(folder_to_process)

#clean up the beginning and the end of the clip
data_frames_to_drop = [(0,200), (-300, -1)]
clean_indicies = np.ones(len(X_train))
for r in data_frames_to_drop:
    clean_indicies[r[0]:r[1]] = 0

S_train = S_train[clean_indicies.astype(np.bool)]
X_train = X_train[clean_indicies.astype(np.bool)]
XX_train = XX_train[clean_indicies.astype(np.bool)]
y_train = y_train[clean_indicies.astype(np.bool)]


print("--done--")



#bring in camera calibration matrix
#with open('fisheye_f1p8_camera_calibration.pkl', 'rb') as f:
#    calibration = pickle.load(f)

#mtx = calibration["mtx"]
#dist = calibration["dist"]

def pipeline(image):
    #undistort it
#     undistort = cv2.undistort(image, mtx, dist, None, mtx)
#     output = undistort[200:361,:,:]
    output = image[100:180,:,:]
    return  output


#reduce image to the road section and apply any transforms in the pipeline
X_train = np.array([pipeline(x) for x in X_train])


#create dataset
xx_train = [] #images
yy_train = [] #actual steering
ss_train = [] #sensors

for current_frame_index in range(len(X_train) - 90):
    frame_start = S_train[current_frame_index]
    frame_end = frame_start + 90
    working_index = current_frame_index
    steering_angles_actual = [y_train[current_frame_index]]
    
    for ii in range(3,90, 3):
        next_img_index = frame_start + ii
        #keep track of the last considered index so in case in exact match is not 
        #found, we have both the index greater and smaller than the desired index
        #and we can fit a line
        last_considered_index = 0
        while S_train[working_index] < next_img_index:
            last_considered_index = working_index
            working_index += 1

        if S_train[working_index] == next_img_index:
            steering_angles_actual.append(y_train[working_index])
        else:
            # fit a linear model between the two points and interpolate the value
            y_1, y_2 = y_train[last_considered_index], y_train[working_index]
            x_1, x_2 = S_train[last_considered_index], S_train[working_index]

            regr = linear_model.LinearRegression()
            regr.fit(np.array([x_1, x_2]).reshape(-1, 1), 
                     np.array([y_1, y_2]).reshape(-1, 1))
            y_interpolated = regr.predict(next_img_index)
            steering_angles_actual.append(y_interpolated[0])

    xx_train.append(X_train[current_frame_index])
    ss_train.append(XX_train[current_frame_index])
    yy_train.append(steering_angles_actual) 
        
yy_train = np.array(yy_train).reshape(-1,30)
xx_train = np.array(xx_train)
ss_train = np.array(ss_train)



data = {
    "images": xx_train,
    "sensors": ss_train,
    "steering_throttle": yy_train
}


with open('V79_run_{}.pkl'.format(folder_to_process), 'wb') as f:
    pickle.dump(data, f)

print("all done, time for training!")
