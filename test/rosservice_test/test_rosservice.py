#!/usr/bin/env python3

import unittest
import json
import rospy

from ros_web_client.message_wrapper import Service



class TestInitialization(unittest.TestCase):

    def test_advertise_service(self):

        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        test_protocol=RosbridgeProtocol(88)

        def outgoing(message):
            print("ROSBRidge Outgoing: {}".format(message))

            test_protocol.incoming('{"op": "service_response", "service": "/test_service", "values": {"value": "88"}, "result": true, "id": "service_request:/test_service:1"}')
        test_protocol.outgoing=outgoing        

        service = Service("/test_service",type="rosapi/GetParam")
        test_protocol.incoming(service.advertise_command())





PKG = 'ros_web_client'
NAME = 'test_initialization'
if __name__ == '__main__':
    rospy.init_node("test_rosbridge", anonymous=False)

    test_case = TestInitialization()
    test_case.test_advertise_service()
    
    
    while not rospy.is_shutdown():
        rospy.spin()
    
    #import rosunit
    #rosunit.unitrun(PKG, NAME, TestInitialization)
