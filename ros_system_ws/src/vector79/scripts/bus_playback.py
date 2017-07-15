#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial

#setup serial port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)




def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    m =  ''.join(data.data.split(" ")[1:])
    rospy.loginfo(rospy.get_caller_id() + '%s', m)
    if(m.startswith("V79-T")):
        throttle_pos = m.split(":")[1]
        ser.write("T{}\n".format(throttle_pos))

    if(m.startswith("V79-S")):
        steer_angle = m.split(":")[1]
        ser.write("S{}\n".format(steer_angle))

def bus_playback():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('bus_playback')
    rospy.loginfo(rospy.get_caller_id() + '%s', "bus plaback started")
    rospy.Subscriber('bus_comm', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    bus_playback()

