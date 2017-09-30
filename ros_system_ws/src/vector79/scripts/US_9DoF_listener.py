#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import threading
import serial
from sensor_msgs.msg import Imu

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
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('US_9DoF_listener')
    
    seq = 0
    while not rospy.is_shutdown():
        info = US_9d0F_serial.readline()
        # message examples ------
        # G:-0.35,-1.01,-4.39
        # A:-0.04,0.09,-1.00
        # M:0.31,0.05,0.01
        #------------------------
        msg = Imu()
        if info:
            message_type = info[0]
            if message_type in ["A","M","G"]:
                message_parts = [float(x) for x in info[2:].split(",")]

                seq += 1
                msg.header.seq = seq
                msg.header.stamp = rospy.Time.now()
                
                #compass
                if message_type == "M":
                    msg.orientation.x = message_parts[0]
                    msg.orientation.y = message_parts[1]
                    msg.orientation.z = message_parts[2]
                    msg.orientation.w = 0

                #accelerometer
                if message_type == "A":
                    msg.angular_velocity.x = message_parts[0]
                    msg.angular_velocity.y = message_parts[1]
                    msg.angular_velocity.z = message_parts[2]

                #gyro
                if message_type == "G":
                    msg.linear_acceleration.x = message_parts[0]
                    msg.linear_acceleration.y = message_parts[1]
                    msg.linear_acceleration.z = message_parts[2]

        
                pub.publish(msg)

if __name__ == '__main__':
    try:
        us9dof_comm()

    except rospy.ROSInterruptException:
        pass

