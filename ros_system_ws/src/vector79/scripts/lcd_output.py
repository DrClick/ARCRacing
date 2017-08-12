#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import String
import serial
import time
import subprocess


def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setblocking(False)
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except:
        return ""

def get_temp():
    return subprocess.check_output(['/opt/vc/bin/vcgencmd','measure_temp']).split('=')[1].rstrip()


#TODO: figure out how to figure out this port programatically
_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=.1)
time.sleep(5)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    
    message_type, message = data.data.split(":")
    write_lcd(message_type, message)

def write_lcd(message_type, message):
    prepend_char = {
        "V79-T":    "T",#TODO: Remove these, legacy from 1st run
        "THR":      "T",
        "V79-S":    "S",#TODO: Remove these, legacy from 1st run
        "STR":      "S",
        "IP":       "I",
        "TMP":      "P",
        "VLT":      "V",
        "M_1":      "1",
        "WRN":      "1",
        "INF":      "2",
        "M_2":      "2",
        "RPM":      "R",
    }

    if message_type in prepend_char:
        output = "{} - {}\n".format(prepend_char[message_type], message)
        print(output)
        _serial.write(output)
        time.sleep(.01)



def lcd_output():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lcd_output', anonymous=True)

    rospy.Subscriber('bus_comm', String, callback)


    write_lcd("M_2", "vector-79 pi ROS online")
    write_lcd("IP", "p" + get_ip_address())
    write_lcd("STR", "0.0")
    write_lcd("THR", "0.0")
    write_lcd("IP", "p" + get_ip_address())
    write_lcd("TMP", "p" + get_temp()[:2])

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lcd_output()

