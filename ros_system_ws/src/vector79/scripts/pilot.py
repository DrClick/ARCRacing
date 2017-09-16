#!/usr/bin/env python
from __future__ import division, print_function


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import tensorflow as tf
from keras.models import Sequential, model_from_json
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers import concatenate, Input
from keras.models import Model
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from scipy.misc import imsave
from scipy.interpolate import interp1d
import pickle
import json

from time import time

#evaluates x on a polynomial curve where the elements are decreasing powers of x 
#[3,5,1,9] is a third degree polynomial in the y = 3x^3 + 5x^2 + 1x^1 + 9x^0
def f_curve(x, curve):
    output = 0
    for i in range(len(curve)):
        output += curve[-i -1] * x**i
    return output

def clamp(x, min_value, max_value):
    return max(min_value, min(max_value, x))


class Pilot:
    def __init__(self):
        self.autonomous_mode = False
        self.cv_bridge = CvBridge()
        self.max_steering_angle = 45
        self.rpm = 0
        self.target_rpm = 1000 #JUMP OFF THE line
        self.rpm_max = 1800
        self.rpm_min = 300
        self.rpm_max_predict = 2000 #after this rpm, look as far foward as possible
        self.num_prediction_frames = 30
        self.throttle_min = 8
        self.throttle_max = 20

        # might turn out to be too much prediction
        self.num_prediction_frames_to_use_rpm = 30
        self.num_prediction_frames_to_use_steer = 6

        # tf
        self.model = self.create_model()
        self.model.load_weights("/home/nvidia/code/ARCRacing/models/at_warehouse_0.h5")
        self.graph = tf.get_default_graph()

        # map how far ahead to look in the prediction for actual steering
        self.__map_rpm_to_prediction_index_steer = interp1d([self.rpm_min, self.rpm_max],
            [3, self.num_prediction_frames_to_use_steer - 1])

        # map how far ahead to look for the curvature of the road to set rpm
        self.__map_rpm_to_prediction_index_rpm = interp1d([0, self.rpm_max_predict],
            [29, self.num_prediction_frames_to_use_rpm - 1])
       
        #angle_to_rpm_curve
        self.set_angle_to_rpm_curve()
        self.set_acceleration_rpm_to_throttle_curve()
        self.set_deceleration_rpm_to_throttle_curve()


        # ros
        rospy.init_node('pilot')
        rospy.Subscriber('/bus_comm', String, self.bus_comm_callback)
        rospy.Subscriber('/car_command', String, self.car_command_callback)
        rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback, queue_size=1)

        self.command_publisher = rospy.Publisher('car_command', String, queue_size=1)
        self.shadow_publisher = rospy.Publisher('shadow_command', String, queue_size=1)
        self.info_publisher = rospy.Publisher('bus_comm', String, queue_size=10)

        self.info_publisher.publish("INF:Enable Auto mode to start pilot")
    

        #camera calibration
        with open('/home/nvidia/code/ARCRacing/models/fisheye_f1p8_camera_calibration_protocol_2.pkl', 'rb') as f:

            calibration = pickle.load(f)
        
        self.mtx = calibration["mtx"]
        self.dist = calibration["dist"]

        self.info_publisher.publish("INF:set auto mode for image prediction")

        rospy.spin()

    def set_angle_to_rpm_curve(self):
        #the precentage of RPM to use
        y_fit = [3500,3200,2952,2566,1963,1527,1346,1209,1156,1000,
         982, 964, 946, 928, 910, 892, 874, 856, 838, 820, 
         802, 784, 766, 748, 730, 712, 694, 676, 658, 640, 
         622, 604, 586, 568, 550, 532, 514, 496, 478, 460, 
         442, 424, 406, 388, 370]
        x_fit = np.arange(45)
        self.angle_to_rpm_curve = np.polyfit(x_fit, y_fit, 6)

    def set_acceleration_rpm_to_throttle_curve(self):
        y_fit = [0, .1, .2, .3, .4,  1]
        x_fit = [0, 100,  500, 750, 1000, 2200]
        self.acceleration_rpm_to_throttle_curve = np.polyfit(x_fit, y_fit, 3)

    def set_deceleration_rpm_to_throttle_curve(self):
        y_fit = [0, 0, .02, .03, .09,  .3,  .5, .8]
        x_fit = [-10, 15,  25, 50, 200, 350, 500, 7000]
        self.deceleration_rpm_to_throttle_curve = np.polyfit(x_fit, y_fit, 3)

    def map_rpm_to_prediction_index_steer(self, rpm):
        return int(self.__map_rpm_to_prediction_index_steer(rpm))

    def map_rpm_to_prediction_index_rpm(self, rpm):
        return int(self.__map_rpm_to_prediction_index_rpm(min(self.rpm_max_predict, rpm)))

    def map_predict_angle_to_target_rpm(self, angle):
        curve = self.angle_to_rpm_curve
        rpm = f_curve(angle, curve)
        clamp(rpm, self.rpm_min, self.rpm_max)
        print("mapped {} angle to {} target rpm".format(angle, rpm))

        return rpm

    def map_decel_rpm_to_throttle(self, rpm):
        curve = self.deceleration_rpm_to_throttle_curve
        throttle = f_curve(abs(rpm), curve) * 100

        #return the breaking between 0 and -100
        throttle =  -clamp(throttle, 0, 100)
        print("mapped {} decelration rpm to {} target throttle".format(rpm, throttle))
        return throttle

    def map_accel_rpm_to_throttle(self, rpm):
        curve = self.acceleration_rpm_to_throttle_curve
        throttle = f_curve(abs(rpm), curve) * self.throttle_max

        #return the breaking between 0 and -100
        throttle = clamp(throttle, self.throttle_min, self.throttle_max)
        print("mapped {} accelration rpm to {} target throttle".format(rpm, throttle))
        return throttle


    # the image pipeline to apply before processing
    def pipeline(self, image):
        return image[100:180,:,:]

    def car_command_callback(self, data):
        # look for autonmouse mode, only 
        # rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        m = data.data

        if (m.startswith("MOD")):
            self.autonomous_mode = True if m.split(":")[1] == "true" else False
        

    """
    listen for the car to be triggered in manual mode and stop making predictions
    """

    def bus_comm_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        m = data.data

        if m == "INF:Manual mode triggered from TX":
            self.autonomous_mode = False

        if m.startswith("RPM"):
            self.rpm = int(m[4:])

    def camera_callback(self, data):
        image_seq_id = data.header.seq
        raw_image = self.cv_bridge.compressed_imgmsg_to_cv2(data, "rgb8")
        predctions = self.predict_steering_angle(raw_image)

        self.set_steering(predctions, image_seq_id)
        self.set_rpm(predctions)
            

    def predict_steering_angle(self, img):
        image_to_predict_on = self.pipeline(img)

        # create a batch of size 1
        predict_image = image_to_predict_on[None, :, :, :]
        predict_rpm = np.array([self.rpm/5000])
        predict_batch = {'image_input': predict_image, 'sensor_input': predict_rpm}

        # prediction should be a value between -1 and 1 so this needs to be denormalized
        

        with self.graph.as_default():
            # get the next set of prediction frames
            prediction = self.model.predict(predict_batch)
            predctions = [x[0][0] * self.max_steering_angle for x in prediction]

        return predctions

    def set_steering(self, predictions, image_seq_id):
        # use the predcition based on RPM, the faster we are going, the further in the 
        # future we need to look
        rpm_to_map = min(max(self.rpm_min, self.rpm), self.rpm_max)
        prediction_index = self.map_rpm_to_prediction_index_steer(rpm_to_map)
        # prediction_index = 5

        predicted_steering_angle = predictions[prediction_index]

        print("Preicted turn angle is: {}".format(predicted_steering_angle))

        # wrte this to the command bus
        message = "STR:{}".format(predicted_steering_angle)
        if self.autonomous_mode:
            self.command_publisher.publish(message)
        else:
            #publish the command to the shadow so we can look for weakness
            message = "{}|{}".format(message, image_seq_id)
            self.shadow_publisher.publish(message)

    def set_rpm(self, predictions):
        # look ahead up the max predicted frames based on linear map of steering angle
        num_look_ahead_frames = self.map_rpm_to_prediction_index_rpm(self.rpm)
        # this curve empahsis large turns and deimphasises very small ones, the inflection
        # point is 18 degrees
        total_steer = sum([(x/4)**2 - (x/16)**4 for x in predictions[:num_look_ahead_frames]])

        #originally I just looked at one frame, the idea here is to 
        #take the total amount of turning into account
        predicted_future_steering_angle = total_steer/num_look_ahead_frames

        #increase this value based on rpm
        rpm_factor = 1 + 6* self.rpm/(self.rpm_max + self.rpm_min)

        predicted_future_steering_angle = rpm_factor * predicted_future_steering_angle
        predicted_future_steering_angle = clamp(predicted_future_steering_angle, 0, 45)

        self.target_rpm = self.map_predict_angle_to_target_rpm(
            abs(min(predicted_future_steering_angle, self.max_steering_angle)))

        # set the throttle based on target RPM
        target_rpm = self.target_rpm
        delta_rpm = self.target_rpm - self.rpm
        abs_delta_rpm = abs(delta_rpm)

        if self.rpm < self.target_rpm:
            target_throttle = self.map_accel_rpm_to_throttle(abs_delta_rpm)
            message = "THR:{}".format(target_throttle)
        else:
            target_throttle = self.map_decel_rpm_to_throttle(abs_delta_rpm)
            message = "THR:{}".format(target_throttle)

        if self.autonomous_mode:
            self.command_publisher.publish(message)
        
    def load_model(self, weights_file):
        
        with open("/home/nvidia/code/ARCRacing/models/model.yaml") as f:
            model = model_from_yaml(f)

        model.compile(loss="mse", optimizer="adam")
        model.load_weights(weights_file)
        
        return model    

    def create_model(self):
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
        
        num_predict_ahead_frames = 30
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


if __name__ == '__main__':
    pilot = Pilot()
