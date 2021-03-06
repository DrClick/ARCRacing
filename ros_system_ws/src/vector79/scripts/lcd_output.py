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
_serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=.1)
time.sleep(5)

def callback(data):
    message_type, message = data.data.split(":")
    write_lcd(message_type, message)

def write_lcd(message_type, message):
    formatting = {
        "THR":      ["T", 4],
        "STR":      ["S", 4],
        "IP":       ["I", 20],
        "TMP":      ["P", 2],
        "VLT":      ["V", 3],
        "M_1":      ["1", 20] ,
        "WRN":      ["1", 20],
        "INF":      ["2", 20],
        "M_2":      ["2", 20],
        "RPM":      ["R", 4],
    }

    if message_type in formatting:
        prepend_char, message_length = formatting[message_type]
        message.strip()
        output = "{} - {}\n".format(prepend_char, message.rjust(message_length))
        if time.time() % 10 == 0 and message_type in ['THR','STR']:
            _serial.write(output)
        else:
            _serial.write(output)
        



def lcd_output():
    rospy.init_node('lcd_output')
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


