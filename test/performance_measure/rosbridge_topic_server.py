#!/usr/bin/env python

import rospy
import time
import csv
import math
from std_msgs.msg import String


pub=None


def callback(msg):
    global pub
    pub.publish(msg.data)


if __name__ == '__main__':
    #Publisher
    pub = rospy.Publisher('test_back', String, queue_size=10)
    rospy.init_node('test_rtt_internal', anonymous=False)
    rate = rospy.Rate(0.5)
    #Subscriber
    rospy.Subscriber("test", String, callback)
    rate.sleep()
    
    while not rospy.is_shutdown():
        rospy.spin()
    
