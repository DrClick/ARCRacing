#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import String
import serial
import time
import subprocess


#TODO: figure out how to figure out this port programatically
_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=.1)
time.sleep(5)


light_state = {
    "brakes": False,
    "left": False,
    "right": False
}


def callback(data):
    message_type, message = data.data.split(":")
    message.strip()

    # write brakes
    if message_type == "THR":
        throttle = int(message)
        if throttle < -8:
            write_light("BRAKE_ON")
            light_state["brakes"] = True
        else:
            if light_state["brakes"]:
                light_state["brakes"] = False
                write_light("BRAKE_OFF")

    # write turn signals
    if message_type == "STR":
        turn = int(message)
        if turn < -10:
            write_light("LEFT_ON")
            light_state["left"] = True
        else:
            if light_state["left"]:
                light_state["left"] = False
                write_light("LEFT_OFF")

        if turn > 10:
            write_light("RIGHT_ON")
            light_state["right"] = True
        else:
            if light_state["right"]:
                light_state["right"] = False
                write_light("RIGHT_OFF")




def write_light(message):
    light_commands = {
        "LEFT_ON":      "RIGHT_OFF\r\nLEFT_ON\r\n",
        "LEFT_OFF":     "LEFT_OFF\r\n",
        "RIGHT":        "LEFT_OFF\r\nRIGHT_ON\r\n",
        "RIGHT_OFF":    "RIGHT_OFF\r\n",
        "BRIGHTS":      "BRIGHT_ON\r\n",
        "RUNNING":      "BRIGHT_OFF\r\nRUNNING_ON\r\n",
        "BRAKE_ON":     "BRAKE_ON\r\n",
        "BRAKE_OFF":    "BRAKE_OFF\r\n",
        "OFF":          "LEFT_OFF\r\nRUNNING_OFF\r\nBRIGHT_OFF\r\nRIGHT_OFF\r\n"
    }


    if message in light_commands:
        command = light_commands[message]
        _serial.write(command)
        



def light_system():
    rospy.init_node('light_system', anonymous=False)
    rospy.Subscriber('bus_comm', String, callback)

    write_light("OFF")
    write_light("RUNNING")

    rospy.spin()

if __name__ == '__main__':
    light_system()


