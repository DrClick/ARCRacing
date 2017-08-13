#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import threading
import serial

print("--------Ultrasonic and 9 Degree of Freedom Listener-----------")
US_9d0F_serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=.1)


bus_codes = {
    "L":    "UFL",  #ultrasonic left
    "R":    "UFR",  #ultrasonic right
    "G":    "GYR",  #gyro
    "A":    "ACC",  #acceleration
    "M":    "MAG",  #magnetic
    "X":    "PRH"   #pitch roll heading
}

def us9dof_comm():
    pub = rospy.Publisher('bus_comm', String, queue_size=10)
    rospy.init_node('US_9DoF_listener')
    
    while not rospy.is_shutdown():
        info = US_9d0F_serial.readline()
        msg = "[{}] {}".format(rospy.get_time(), info)

        
        try:
            raw_code, msg = info.split(":")
            if raw_code in bus_codes:
                msg = "{}:{}".format(bus_codes[raw_code], msg)
                pub.publish(msg)
        except:
            pass

if __name__ == '__main__':
    try:
        us9dof_comm()

    except rospy.ROSInterruptException:
        pass

