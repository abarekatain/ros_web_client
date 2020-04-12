#!/usr/bin/env python3

import unittest
import json
import rospy

from ros_web_client.initialization import ConfigParser
from ros_web_client.message_wrapper import Param

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

class TestInitialization(unittest.TestCase):

    def test_config_parser(self):
        with open('sample_config.json', 'r') as outfile:
            sample_config = outfile.read()

        parser = ConfigParser(sample_config)
        parser.parse_topics()
        self.assertTrue(True)
    
    def test_getparam_service(self):
        rospy.set_param('/test_param', 88)
        param = Param("/test_param")
        test_protocol=RosbridgeProtocol(88)

        def outgoing(message):
            print(message)
        
        test_protocol.outgoing=outgoing

        test_protocol.incoming(param.get_command())
        #print(param.get_command())






PKG = 'ros_web_client'
NAME = 'test_initialization'
if __name__ == '__main__':
    test_case = TestInitialization()
    #test_case.test_config_parser()
    test_case.test_getparam_service()
    
    
    
    #import rosunit
    #rosunit.unitrun(PKG, NAME, TestInitialization)
