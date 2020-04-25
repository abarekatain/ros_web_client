#!/usr/bin/env python3

import unittest
import json
import rospy

from ros_web_client.message_wrapper import Topic


class TestInitialization(unittest.TestCase):

    def test_scan_topic(self):
        with open('sample_scan.txt', 'r') as outfile:
            sample_scan = outfile.read()  

        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        test_protocol=RosbridgeProtocol(88)

        def outgoing(message):
            print("ROSBRidge Outgoing: {}".format(message))

        test_protocol.outgoing=outgoing        

        topic=Topic("/scan","sensor_msgs/LaserScan",latch=False)
        test_protocol.incoming(topic.advertise_command())

        test_protocol.incoming(sample_scan)




PKG = 'ros_web_client'
NAME = 'test_initialization'
if __name__ == '__main__':
    rospy.init_node("test_rosbridge", anonymous=False)

    test_case = TestInitialization()
    test_case.test_scan_topic()
    
    
    while not rospy.is_shutdown():
        rospy.spin()
    
    #import rosunit
    #rosunit.unitrun(PKG, NAME, TestInitialization)
