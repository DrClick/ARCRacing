#!/usr/bin/python

# Start up ROS pieces.
import roslib
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import sys

class BagExtractor():
    camera_topic = "camera/image/compressed"
    shadow_topic = "shadow_command"
    data_topic = "bus_comm"
    output = []
    image_counter = 0
    current_values = {
        "STR": 0,
        "RPM": 0,
        "THR": 0,
        "UFR": 0,
        "UFL": 0,
        "ACC": [0,0,0],
        "GYR": [0,0,0],
        "MAG": [0,0,0],
        "PRH": [0,0,0]
    }

    #a places to store found predictions for model shadowing
    predictions = [] 

    def __init__(self):
        self.output_dir = os.path.join(sys.path[0], sys.argv[1])
        self.bag_file = os.path.join(sys.path[0], sys.argv[2])
        rospy.loginfo("Output directory = %s", self.output_dir)
        rospy.loginfo("Bag filename = %s", self.bag_file)

        self.bridge = CvBridge()
        columns = ["image_id"]
        columns.extend(self.current_values.keys())
        columns.append("time")

        self.output.append("|".join(columns))

    def log_prediction(self, msg):
        print(msg)
        message, image_seq_id = msg.data.split("|")
        prediction = round(float(message.split(":")[1]),3)
        self.predictions.append((image_seq_id, prediction))

    def extract_and_sync(self):
        # Open bag file.
        with rosbag.Bag(self.bag_file, 'r') as bag:
            print("Opened bag file just fine!")
            for topic, msg, t in bag.read_messages():
                sys.stdout.write(".")
                topic_parts = topic.split('/')
                
#                if topic_parts[0] == 'bus_comm':
#                    bus_code, bus_value = msg.data.split(":")
#                    if bus_code in self.current_values:
#                        if bus_code != "RPM":
#                           self.current_values[bus_code] = bus_value
#
                if topic_parts[0] == 'car_velocity':
                    steer = msg.angular.z
                    throttle = msg.linear.x
                    self.current_values["STR"] = steer
                    self.current_values["THR"] = throttle

                if topic_parts[0] == 'shadow_command':
                    self.log_prediction(msg)

                if len(topic_parts) < 2:
                    continue       

                if topic_parts[1] == 'car_rpm':
                    self.current_values["RPM"] = msg.data



                if topic_parts[1] == 'camera' and topic_parts[2] == "image":
                    #extract the image id
                    image_seq_id = msg.header.seq

                    #grap all the current values, sync method is last input
                    values = [str(x).strip() for x in self.current_values.values()]
                    entry = "|".join([str(image_seq_id)] + values + [str(t.to_nsec())])
                    self.output.append(entry)
                    
                    #save the image
                    cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                    image_name = "{}/{}.jpg".format(self.output_dir, image_seq_id)
                    cv2.imwrite(image_name, cv_image)




        #write the outputs
        with open(self.output_dir + "/_data.csv", 'w') as f:
            for i in self.output:
                f.write("{}\n".format(i))

        with open(self.output_dir + "/_predictions.csv", 'w') as f:
            for i in self.predictions:
                f.write("{},{}\n".format(i[0], i[1]))

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("bag_extractor")
    # Go to class functions that do all the heavy lifting. Do error checking.
    
    bagExtractor = BagExtractor()
    bagExtractor.extract_and_sync()

