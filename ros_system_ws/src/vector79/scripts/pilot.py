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

import pickle


class Pilot:
    def __init__(self):
        self.autonomous_mode = False
        self.cv_bridge = CvBridge()
        self.flag_1 = True

        rospy.init_node('pilot')
        rospy.Subscriber('/bus_comm', String, self.bus_comm_callback)
        rospy.Subscriber('/car_command', String, self.car_command_callback)
        rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback, queue_size = 1)

        self.command_publisher = rospy.Publisher('car_command', String, queue_size=10000)

        rospy.loginfo("Enable Auto mode to start pilot")
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    

    #the image pipeline to apply before processing
    def pipeline(self, img):
        output = img[200:361,:,:]
        return  cv2.resize(output, (0,0), fx=0.5, fy=0.5)


    def car_command_callback(self, data):
        #look for autonmouse mode, only 
        rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        m = data.data

        if(m.startswith("MOD")):
            self.autonomous_mode = True if m.split(":")[1] == "true" else False
            print("set auto mode for image prediction", self.autonomous_mode)

    """
    listen for the car to be triggered in manual mode and stop making predictions
    """
    def bus_comm_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        m = data.data

        if m =="INF:Manual mode triggered from TX":
            self.autonomous_mode = False
        


    def camera_callback(self, data):
        if not self.autonomous_mode:
            return

        raw_image = self.cv_bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        image_to_predict_on = self.pipeline(raw_image)

        #create a batch of size 1
        predict_batch = image_to_predict_on[None,:,:,:]

        # prediction should be a value between -1 and 1 so this needs to be denormalized
        max_steering_angle = 45

        global graph
        with graph.as_default():

            predicted_steering_angle = int(model.predict(predict_batch)[0][0] * max_steering_angle)

        #wrte this to the command bux
        message = "STR:{}".format(predicted_steering_angle)
        print(message)
        self.command_publisher.publish(message)

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
model.load_weights("//home/nvidia/code/ARCRacing/ros_system_ws/src/vector79/scripts/model.h5")
graph = tf.get_default_graph()


if __name__ == '__main__':
    pilot = Pilot()
