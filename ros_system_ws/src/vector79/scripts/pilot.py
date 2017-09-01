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
from keras.layers import concatenate, Input
from keras.models import Model
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from scipy.misc import imsave
from scipy.interpolate import interp1d
import pickle

from time import time


class Pilot:
    def __init__(self):
        self.autonomous_mode = False
        self.cv_bridge = CvBridge()
        self.max_steering_angle = 45
        self.rpm = 0
        self.target_rpm = 500
        self.max_rpm = 1500
        self.min_rpm = 100
        self.num_prediction_frames = 30

        # might turn out to be too much prediction
        self.num_prediction_frames_to_use_rpm = 30
        self.num_prediction_frames_to_use_steer = 10

        # tf
        self.model = self.create_model()
        self.model.load_weights("/home/nvidia/code/ARCRacing/models/office_set_predict_ahead_90_19.h5")
        self.graph = tf.get_default_graph()

        # maps
        self.__map_rpm_to_prediction_index = interp1d([self.min_rpm, self.max_rpm],
            [0, self.num_prediction_frames_to_use_steer - 1])
        # self.__map_predict_angle_to_target_rpm = interp1d([0, 45], [self.max_rpm, self.min_rpm])
        self.__map_decel_rpm_to_throttle = interp1d([0, self.max_rpm - self.min_rpm], [-20, -100.0])
        self.__map_accel_rpm_to_throttle = interp1d([0, self.max_rpm], [8.0, 15.0])

        #angle_to_rpm_curve
        self.set_angle_to_rpm_curve()


        # ros
        rospy.init_node('pilot')
        rospy.Subscriber('/bus_comm', String, self.bus_comm_callback)
        rospy.Subscriber('/car_command', String, self.car_command_callback)
        rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback, queue_size=1)

        self.command_publisher = rospy.Publisher('car_command', String, queue_size=10)
        self.info_publisher = rospy.Publisher('bus_comm', String, queue_size=10)

        self.info_publisher.publish("INF:Enable Auto mode to start pilot")
        rospy.spin()

    def set_angle_to_rpm_curve(self):
        throttle_curve = [
            (0, self.max_rpm), # at 0
            (1, self.max_rpm * .99),
            (10, self.max_rpm * .9),
            (15, self.max_rpm * .6),
            (35, self.min_rpm * 1.1),
            (40, self.min_rpm),
            (45, self.min_rpm)
        ]
        y = [x[1] for x in throttle_curve]
        X = [x[0] for x in throttle_curve]
        self.angle_to_rpm_curve = np.polyfit(X, y, 3)

    def map_rpm_to_prediction_index(self, rpm):
        return self.__map_rpm_to_prediction_index(rpm)

    def map_predict_angle_to_target_rpm(self, angle):
        curve = self.angle_to_rpm_curve
        rpm = curve[0]*angle**3 + curve[1]*angle**2 + curve[2]*angle + curve[3]
        rpm = min(self.max_rpm, rpm)
        rpm = max(self.min_rpm, rpm)

        return rpm

    def map_decel_rpm_to_throttle(self, rpm):
        return self.__map_decel_rpm_to_throttle(rpm)

    def map_accel_rpm_to_throttle(self, rpm):
        return self.__map_accel_rpm_to_throttle(rpm)


    # the image pipeline to apply before processing
    def pipeline(self, img):
        output = img[100:180, :, :]
        return output

    def car_command_callback(self, data):
        # look for autonmouse mode, only 
        # rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        m = data.data

        if (m.startswith("MOD")):
            self.autonomous_mode = True if m.split(":")[1] == "true" else False
            self.info_publisher.publish("INF:set auto mode for image prediction")

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
        raw_image = self.cv_bridge.compressed_imgmsg_to_cv2(data, "rgb8")
        predctions = self.predict_steering_angle(raw_image)

        if self.autonomous_mode:
            self.set_steering(predctions)
            self.set_rpm(predctions)

    def predict_steering_angle(self, img):
        image_to_predict_on = self.pipeline(img)

        # create a batch of size 1
        predict_batch = image_to_predict_on[None, :, :, :]

        # prediction should be a value between -1 and 1 so this needs to be denormalized
        

        with self.graph.as_default():
            # get the next set of prediction frames
            prediction = self.model.predict(predict_batch)
            predctions = [x[0][0] * self.max_steering_angle for x in prediction]

        return predctions

    def set_steering(self, predctions):
        # use the predcition based on RPM, the faster we are going, the further in the 
        # future we need to look
        rpm_to_map = min(max(self.min_rpm, self.rpm), self.max_rpm)
        print("rpm to map", rpm_to_map)
        prediction_index = int(self.map_rpm_to_prediction_index(rpm_to_map))
        predicted_steering_angle = predctions[prediction_index]

        # wrte this to the command bus
        message = "STR:{}".format(predicted_steering_angle)
        self.command_publisher.publish(message)

    def set_rpm(self, predictions):
        # the last predicted frame is what we use to set the target rpm
        predicted_future_steering_angle = predictions[self.num_prediction_frames_to_use_rpm - 1]

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

        print(message)
        self.command_publisher.publish(message)

    def create_model(self):
        model = Sequential()

        image_input = Input(shape=(80, 320, 3), name='image_input', dtype='float32')
        # preprocess
        X = Lambda(lambda x: x / 255.0 - 0.5, input_shape=(80, 320, 3), name="lambda_1")(image_input)

        # conv1 layer
        X = Convolution2D(32, (5, 5), name="conv_1")(X)
        X = MaxPooling2D((2, 2), name="pool_1")(X)
        X = Activation('relu', name="relu_1")(X)

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

        # add fully connected layers
        X = Flatten(name="flat_1")(X)

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

        # outputs are the next 10 frames
        steer_outputs = []
        for i in range(self.num_prediction_frames):
            steer_output = Dense(1, name='steer_output_{}'.format(i))(X)
            steer_outputs.append(steer_output)

        # model = Model(inputs=[image_input, sensor_input], outputs=[steer_output, throttle_output])
        model = Model(inputs=[image_input], outputs=steer_outputs)

        loss_def = {"steer_output_{}".format(i): "mse" for i in range(self.num_prediction_frames)}
        loss_weight_def = {"steer_output_{}".format(i): 1.0 for i in range(self.num_prediction_frames)}

        # note, setting the loss weight to favor steering
        model.compile(optimizer='adam', loss=loss_def, loss_weights=loss_weight_def)

        return model


if __name__ == '__main__':
    pilot = Pilot()
