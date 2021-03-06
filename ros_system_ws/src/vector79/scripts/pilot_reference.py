#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import tensorflow as tf
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from scipy.misc import imsave
import pickle

from time import time


class Pilot:
    def __init__(self):
        self.autonomous_mode = False
        self.cv_bridge = CvBridge()
        self.rpm = -1

        rospy.init_node('pilot')
        rospy.Subscriber('/bus_comm', String, self.bus_comm_callback)
        rospy.Subscriber('/car_command', String, self.car_command_callback)
        rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback, queue_size = 1)

        self.command_publisher = rospy.Publisher('car_command', String, queue_size=10)

        rospy.loginfo("Enable Auto mode to start pilot")
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    

    #the image pipeline to apply before processing
    def pipeline(self, img):
        output = img[100:180,:,:]
        # output = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
        return output
        # return  cv2.resize(output, (0,0), fx=0.5, fy=0.5)


    def car_command_callback(self, data):
        #look for autonmouse mode, only 
        #rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        m = data.data

        if(m.startswith("MOD")):
            self.autonomous_mode = True if m.split(":")[1] == "true" else False
            print("set auto mode for image prediction", self.autonomous_mode)


    """
    listen for the car to be triggered in manual mode and stop making predictions
    """
    def bus_comm_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        m = data.data

        if m =="INF:Manual mode triggered from TX":
            self.autonomous_mode = False
            self.rpm = -1


        if not self.autonomous_mode:
            return
        # try and set rpms to 100. If its above 100, set throttle to zero, if below set to 8
        #slow down for corners
        if m.startswith("RPM"):
            self.rpm = int(m[4:])
            print("RPM", self.rpm)
            target_rpm = 300

            if self.rpm < target_rpm:
                message = "THR:8"
                self.command_publisher.publish(message)
            else:
                message = "THR:0"
                self.command_publisher.publish(message)


    def camera_callback(self, data):
        if not self.autonomous_mode:
            return


        raw_image = self.cv_bridge.compressed_imgmsg_to_cv2(data, "rgb8")
        image_to_predict_on = self.pipeline(raw_image)
        #imsave("predict.jpg", image_to_predict_on)


        #create a batch of size 1
        predict_batch = image_to_predict_on[None,:,:,:]

        # prediction should be a value between -1 and 1 so this needs to be denormalized
        max_steering_angle = 45

        global graph
        with graph.as_default():

            predicted_steering_angle = model.predict(predict_batch)[0][0] * max_steering_angle

        #wrte this to the command bux
        message = "STR:{}".format(predicted_steering_angle)
        self.command_publisher.publish(message)


        #start the car rolling
        if self.rpm == -1:
            self.rpm = 0
            self.command_publisher.publish("THR:8")
        
        

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

model = create_model()
model.load_weights("/home/nvidia/code/ARCRacing/models/office_set_1_8.h5")
graph = tf.get_default_graph()


if __name__ == '__main__':
    pilot = Pilot()

