#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import PyCmdMessenger
import threading

# Initialize an ArduinoBoard instance.  This is where you specify baud rate and
# serial timeout.  If you are using a non ATmega328 board, you might also need
# to set the data sizes (bytes for integers, longs, floats, and doubles).  
arduino = PyCmdMessenger.ArduinoBoard("/dev/ttyACM0", baud_rate=115200)

commands = [
    ["cmd_steer", "i"],
    ["cmd_throttle", "i"],
    ["cmd_rpm", "i"],
    ["cmd_sonar", "ii"],
    ["cmd_toggle_ebrake", "?"],
    ["cmd_govern_forward", "i"],
    ["cmd_govern_reverse", "i"],
    ["cmd_set_mode", "?"],
    ["cmd_set_steer_bias", "i"],
    ["cmd_info", "s"]
]
# Initialize the messenger
commander = PyCmdMessenger.CmdMessenger(arduino, commands)

print("--------PLAYBACK-----------")


def callback(data):
    print('PLAYBACK RCVD:', data)
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    m =  ''.join(data.data.split(" ")[1:])
    rospy.loginfo(rospy.get_caller_id() + '%s', m)
    if(m.startswith("V79-T")):
        throttle_pos = int(m.split(":")[1])
        commander.send("cmd_throttle", throttle_pos)

    if(m.startswith("V79-S")):
        steer_angle = int(m.split(":")[1])
        commander.send("cmd_steer", steer_angle)

def read_from_pi(_commander):
    while True:
        print(_commander.receive())


def bus_playback():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('bus_playback')
    rospy.loginfo(rospy.get_caller_id() + '%s', "bus playback started")
    rospy.Subscriber('bus_comm', String, callback)

    
    listener_thread = threading.Thread(target = read_from_pi, args=[commander])
    print "Starting listener"
    listener_thread.start()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    bus_playback()

