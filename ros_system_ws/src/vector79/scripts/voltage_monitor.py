#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
import time
import subprocess


def voltage_monitor():
    rospy.init_node('voltage_monitor')
    info_pub = rospy.Publisher('bus_comm', String, queue_size=1)
    voltage_pub = rospy.Publisher('voltage', Float32, queue_size=1)


    while True:
        time.sleep(10)
        input_voltage = subprocess.check_output(['cat','/sys/bus/i2c/devices/0-0040/iio_device/in_voltage0_input']).rstrip()
        voltage = round(float(input_voltage)/1000,2)
        msg = "VLT:{}".format(voltage)
        rospy.loginfo(rospy.get_caller_id() + '%s', msg)
        info_pub.publish(msg)
        voltage_pub.publish(voltage)

        if voltage < 11.2:
            msg = "WRN:LOW-VOLTAGE {}".format(voltage)
            info_pub.publish(msg)

if __name__ == '__main__':
    voltage_monitor()


