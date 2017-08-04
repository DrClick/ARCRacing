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
    data_topic = "bus_comm"
    output = []
    image_counter = 0
    current_steer = 0
    current_throttle = 0

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.output_dir = os.path.join(sys.path[0], sys.argv[1])
        self.bag_file = os.path.join(sys.path[0], sys.argv[2])
        rospy.loginfo("Output directory = %s", self.output_dir)
        rospy.loginfo("Bag filename = %s", self.bag_file)

        self.bridge = CvBridge()

    def extract_and_sync(self):
        # Open bag file.
        with rosbag.Bag(self.bag_file, 'r') as bag:
            print("Opened bag file just fine!")
            for topic, msg, t in bag.read_messages():
                # print("found msg", topic, t)
                topic_parts = topic.split('/')
                
                #read in the sensor data
                if topic_parts[1] == 'bus_comm':
                    steer_index = msg.data.find("V79-S:")
                    if steer_index > -1:
                        self.current_steer = msg.data[steer_index + 6:].strip()

                    throttle_index = msg.data.find("V79-T:")
                    if throttle_index > -1:
                        self.current_throttle = msg.data[throttle_index + 6:].strip()

                if topic_parts[1] == 'camera' and topic_parts[2] == "image":
                    entry = [str(self.image_counter), str(self.current_throttle), str(self.current_steer), str(t.to_nsec())]
                    self.output.append(entry)
                    
                    try:
                        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError, e:
                        print e
                    image_name = "{}/{}_{}.jpg".format(self.output_dir, self.image_counter,t.to_nsec())
                    cv2.imwrite(image_name, cv_image)
                    self.image_counter += 1




        #write the output
        with open(self.output_dir + "/_data.csv", 'w') as f:
            for i in self.output:
                print(i)
                f.write("{}\n".format(",".join(i)))
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("bag_extractor")
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        bagExtractor = BagExtractor()
        bagExtractor.extract_and_sync()

    except rospy.ROSInterruptException: pass
