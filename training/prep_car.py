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
    X = [] #images
    XX = [] #senor data (aka speed in this case)
    y = [] #actual steering angles (throttle in the future)

    
    with open('{}/_data.csv'.format(folder_to_process)) as f:
        reader = csv.reader(f)
        print(''.join(next(reader)))
        for line in reader:
            line = ''.join(line).split('|')
            img_file= "{}".format(int(line[0]))
            img = imread('{}/{}.jpg'.format(folder_to_process, img_file))
            
            
            X.append(img)
            #image_id|ACC|MAG|UFL|GYR|STR|UFR|RPM|THR|PRH|time
            print(line, len(line))
            XX.append([float(line[7])/5000, float(line[8])/100]) 
            y.append([float(line[5])/45.0]) # steering is -45 to 45

    return (np.array(X), np.array(XX), np.array(y))


X_train, XX_train, y_train = extract_data(folder_to_process)

print("Done reading in data")



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


#drop bad frames

data_frames_to_drop = [(0,200), (-300, -1)]


clean_indicies = np.ones(len(X_train))
for r in data_frames_to_drop:
    clean_indicies[r[0]:r[1]] = 0


X_cleaned = X_train[clean_indicies.astype(np.bool)]
XX_cleaned = XX_train[clean_indicies.astype(np.bool)]
y_cleaned = y_train[clean_indicies.astype(np.bool)]


data = {
    "images": X_cleaned,
    "sensors": XX_cleaned,
    "steering_throttle": y_cleaned
}


with open('V79_run_{}.pkl'.format(folder_to_process), 'wb') as f:
    pickle.dump(data, f)

print("all done, time for training!")
