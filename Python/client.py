#!/usr/bin/env python2

import rospy
from std_srvs.srv import SetBool, SetBoolRequest

# Node setup
rospy.init_node("client")

# Define client and wait for service
client = rospy.ServiceProxy("test_service", SetBool)
client.wait_for_service()

# Create request message
request = SetBoolRequest()
request.data = True

# Receive response and store it
response = client(request)

# Visualize
print(response)
