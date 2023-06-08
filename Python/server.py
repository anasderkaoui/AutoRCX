#!/usr/bin/env python2

import rospy
from std_srvs.srv import SetBool, SetBoolResponse


def callback(request):
    # Create response message
    response = SetBoolResponse()
    # Execute task
    if request.data == True:
        response.success = True
        response.message = "Device enabled"
    else:
        response.success = True
        response.message = "Device disabled"
    # Return response
    return response

# Node setup
rospy.init_node("server")
rospy.Service("test_service", SetBool, callback)

rospy.spin()
