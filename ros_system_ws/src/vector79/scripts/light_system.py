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
    "right": False,
    "auto": False,
    "pilot": False,
    "warn": False
}


def callback(data):
    message_type, message = data.data.split(":")
    message.strip()

    if message_type == "INF":
        print(message)
        if message == "Entering Auto mode":
            light_state["auto"] = True
            write_light("GREEN_ON")
            write_light("RED_OFF")
            write_light("BLUE_ON")

        if message == "Enable Auto mode to start pilot":
            light_state["pilot"] = True
            light_state["auto"] = True
            write_light("GREEN_OFF")
            write_light("BLUE_ON")
            write_light("RED_OFF")


        if message == "Manual mode triggered from TX":
            light_state["auto"] = False
            light_state["pilot"] = False
            write_light("GREEN_ON")
            write_light("RED_OFF")
            write_light("BLUE_OFF")

    if message_type == "WRN":
        light_state["warn"] = True
        write_light("RED_ON")
        write_light("BLUE_OFF")
        write_light("GREEN_OFF")


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
        "RIGHT_ON":     "LEFT_OFF\r\nRIGHT_ON\r\n",
        "RIGHT_OFF":    "RIGHT_OFF\r\n",
        "BRIGHTS":      "BRIGHT_ON\r\n",
        "RUNNING":      "BRIGHT_OFF\r\nRUNNING_ON\r\n",
        "BRAKE_ON":     "BRAKE_ON\r\n",
        "BRAKE_OFF":    "BRAKE_OFF\r\n",
        "GREEN_ON":     "STATUS_ON\r\nGREEN_ON\r\n",
        "GREEN_OFF":    "GREEN_OFF\r\n",
        "RED_ON":       "STATUS_ON\r\nRED_ON\r\n",
        "RED_OFF":      "RED_OFF\r\n",
        "BLUE_ON":      "STATUS_ON\r\nBLUE_ON\r\n",
        "BLUE_OFF":     "BLUE_OFF\r\n",
        "OFF":          "LEFT_OFF\r\nRUNNING_OFF\r\nBRIGHT_OFF\r\nRIGHT_OFF\r\n"
    }


    if message in light_commands:
        command = light_commands[message]
        _serial.write(command)
        



def light_system():
    rospy.init_node('light_system')
    rospy.Subscriber('bus_comm', String, callback)

    write_light("OFF")
    time.sleep(1)
    write_light("RUNNING")
    time.sleep(1)
    write_light("GREEN_ON")
    time.sleep(1)
    write_light("GREEN_OFF")
    write_light("BLUE_ON")
    time.sleep(1)
    write_light("BLUE_OFF")
    write_light("RED_ON")
    time.sleep(1)
    write_light("RED_OFF")

    rospy.spin()

if __name__ == '__main__':
    light_system()


