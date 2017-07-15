#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial

#setup serial port
ser = serial.Serial('/dev/ttyACM0')


def bus_comm():
    pub = rospy.Publisher('bus_comm', String, queue_size=10)
    rospy.init_node('bus_comm')
    # rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        car_info = ser.readline()
        msg = "[{}] {}".format(rospy.get_time(), car_info)
        # do i really need to do this
        # rospy.loginfo(msg)


        pub.publish(msg)

if __name__ == '__main__':
    try:
        bus_comm()
    except rospy.ROSInterruptException:
        pass

