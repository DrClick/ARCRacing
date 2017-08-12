#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import  PyCmdMessenger
from ArduinoBoard import ArduinoBoard
import threading

# Initialize an ArduinoBoard instance.  This is where you specify baud rate and
# serial timeout.  If you are using a non ATmega328 board, you might also need
# to set the data sizes (bytes for integers, longs, floats, and doubles).  
arduino = ArduinoBoard("/dev/ttyUSB0", baud_rate=115200, timeout=1)

commands = [
    ["cmd_steer", "i"],             # TO / FROM CAR
    ["cmd_throttle", "i"],          # TO / FROM CAR
    ["cmd_rpm", "i"],               # FROM CAR
    ["cmd_sonar", "ii"],            # FROM CAR
    ["cmd_toggle_ebrake", "?"],     # TO CAR
    ["cmd_govern_forward", "i"],    # TO CAR
    ["cmd_govern_reverse", "i"],    # TO CAR
    ["cmd_set_mode", "?"],          # TO CAR
    ["cmd_set_steer_bias", "i"],    # TO CAR
    ["cmd_info", "s"],              # FROM CAR
    ["cmd_voltage", "d"]            # FROM CAR
]

commands_to_bus_code = {
    "cmd_steer":        "STR",
    "cmd_throttle":     "THR",
    "cmd_rpm":          "RPM",
    "cmd_sonar":        "USR",
    "cmd_info":         "INF",
    "cmd_voltage":      "VLT"
}
# Initialize the messenger
commander = PyCmdMessenger.CmdMessenger(arduino, commands)

print("--------CAR COMMUNICATOR-----------")


def cmd_callback(data):
    print('Car command RCVD:', data)
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    m =  data.data

    if(m.startswith("THR")):
        throttle_pos = int(m.split(":")[1])
        commander.send("cmd_throttle", throttle_pos)

    if(m.startswith("STR")):
        steer_angle = int(m.split(":")[1])
        commander.send("cmd_steer", steer_angle)

    if(m.startswith("MOD")):
        mode = True if m.split(":")[1] == "true" else False
        commander.send("cmd_set_mode", mode)

def read_from_pi(_commander):
    pub = rospy.Publisher('bus_comm', String, queue_size=10000)
    while True:
        # publish the commands received if a command was received
        raw_command = commander.receive()
        if raw_command is None:
            continue

        command, values, time = raw_command

        if command in commands_to_bus_code:
            bus_code = commands_to_bus_code[command]
            msg_values = ",".join([str(x) for x in values])
            message = "{}:{}".format(bus_code, msg_values)
            print(message)
            pub.publish(message)


def car_communicator():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('car_communicator')
    rospy.loginfo(rospy.get_caller_id() + '%s', "car communication started")
    rospy.Subscriber('car_command', String, cmd_callback)

    
    listener_thread = threading.Thread(target = read_from_pi, args=[commander])
    print "Starting car communicator"
    listener_thread.start()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    car_communicator()

