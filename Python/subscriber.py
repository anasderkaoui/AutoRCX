#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int64

# Callback function
def callback(msg):
    print(msg)

# Node + Subscriber initialization
rospy.init_node("subscriber")
rospy.Subscriber("topic", Int64, callback) # callback is a function, can be named otherwise but it is a callback function

# Spinning
rospy.spin()
