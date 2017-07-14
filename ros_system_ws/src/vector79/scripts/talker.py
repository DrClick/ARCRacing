#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial

#setup serial port
ser = serial.Serial('/dev/ttyACM0')

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        car_info = ser.readline()
        msg = "[{}] {}".format(rospy.get_time(), car_info)
        rospy.loginfo(msg)
        pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

