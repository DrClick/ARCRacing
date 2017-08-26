#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial

#setup serial port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)


def bus_comm():
    pub = rospy.Publisher('bus_comm', String, queue_size=10)
    rospy.init_node('bus_comm')

    while not rospy.is_shutdown():
        car_info = ser.readline()
        msg = "[{}] {}".format(rospy.get_time(), car_info)
        pub.publish(msg)

if __name__ == '__main__':
    try:
        bus_comm()
    except rospy.ROSInterruptException:
        pass

