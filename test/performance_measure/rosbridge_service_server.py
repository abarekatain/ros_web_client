#!/usr/bin/env python

import rospy
import time
import csv
import math
from ros_web_client.srv import RTTSrv,RTTSrvRequest,RTTSrvResponse





def callback(req):
    return RTTSrvResponse(req.message)


if __name__ == '__main__':

    rospy.init_node('test_rtt_internal', anonymous=False)
    s = rospy.Service('/test', RTTSrv, callback)
    
    while not rospy.is_shutdown():
        rospy.spin()
    
