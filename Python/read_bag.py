#!/usr/bin/env python2

import rospy
import rosbag

rospy.init_node("read_bag")
bag = rosbag.Bag("/home/nvidiarobocar/catkin_ws/src/pub_sub/bag/2023-06-08-18-50-35.bag")

for topic, msg, t in bag.read_messages(topics = ["/topii", "/johan"]):
    if topic == "/topii":
        print(" /topii's data is : ")
        print(msg.data)
        print(t) # Time stamp in nano seconds
    if topic == "/johan":
        print("johan's data is: ")
        print(msg.data)
        print(t, "in nanosec")
