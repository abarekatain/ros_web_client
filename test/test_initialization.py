#!/usr/bin/env python3

import unittest
import json
import rospy

from ros_web_client.initialization import ConfigParser,ServiceHandler
from ros_web_client.message_wrapper import Param,Topic

from twisted.internet.defer import inlineCallbacks


class TestInitialization(unittest.TestCase):

    def test_config_parser(self):
        with open('sample_config.json', 'r') as outfile:
            sample_config = outfile.read()

        parser = ConfigParser(sample_config)
        commands= parser.parse()
        print(*commands)
    
    def test_getparam_service(self):
        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        if not rospy.has_param("/test_param"):
            rospy.set_param('/test_param', 88)
        param = Param("/test_param")
        test_protocol=RosbridgeProtocol(88)

        def outgoing(message):
            print("ROSBRidge Outgoing: {}".format(message))
        
        test_protocol.outgoing=outgoing

        test_protocol.incoming(param.get_command())

    def test_setparam_service(self):
        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        param = Param("/test_param")
        test_protocol=RosbridgeProtocol(88)

        def outgoing(message):
            print("ROSBRidge Outgoing: {}".format(message))
        
        test_protocol.outgoing=outgoing

        test_protocol.incoming(param.set_command(885))
    
    def test_rosbridge_false_subscription(self):
        from std_msgs.msg import String
        pub = rospy.Publisher("/test", String, queue_size=10,latch=True)
        pub.publish("dada")

        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        test_protocol=RosbridgeProtocol(88)

        def outgoing(message):
            print("ROSBRidge Outgoing: {}".format(message))
        
        test_protocol.outgoing=outgoing

        topic=Topic("/test","sensor_msgs/Image")
        test_protocol.incoming(topic.advertise_command())



    def test_list_json_conversion(self):
        test_list=["command1", "command2", "command3"]        
        json_str = json.dumps(test_list)
        list_again = json.loads(json_str)
        print(list_again[0])
    
    def test_command_handler_client_side(self):
        test_list = []

    def test_rosbridgeprotocol(self):
        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        test_protocol=RosbridgeProtocol(88)
        
        def outgoing(message):
            print("ROSBRidge Outgoing: {}".format(message))
        
        test_protocol.outgoing=outgoing

        with open('sample_config.json', 'r') as outfile:
            sample_config = outfile.read()

        parser = ConfigParser(sample_config)
        commands = parser.parse()

        test_protocol.incoming(commands[1])
    @inlineCallbacks
    def test_service_handler(self):
        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        test_protocol=RosbridgeProtocol(88)

        service_handler=ServiceHandler(test_protocol)

        def outgoing(message):
            service_handler.callback(message)
            print("ROSBRidge Outgoing: {}".format(message))
        
        test_protocol.outgoing=outgoing
        

        if not rospy.has_param("/test_param"):
            rospy.set_param('/test_param', 88)
        param = Param("/test_param") 
        result = yield service_handler.execute(param.get_command())  
        print("result is {}".format(result))     







PKG = 'ros_web_client'
NAME = 'test_initialization'
if __name__ == '__main__':
    rospy.init_node("test_node", anonymous=False)


    test_case = TestInitialization()
    test_case.test_getparam_service()
    
    
    while not rospy.is_shutdown():
        rospy.spin()
    
    #import rosunit
    #rosunit.unitrun(PKG, NAME, TestInitialization)
