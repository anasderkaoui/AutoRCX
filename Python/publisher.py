#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int64

# Node + Topic initialization
rospy.init_node("publisher_node")
pub = rospy.Publisher("topic", Int64, queue_size = 1)

# Looping
while not rospy.is_shutdown():
    pub.publish(1)
    rospy.sleep(1)
