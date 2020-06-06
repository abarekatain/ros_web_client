#!/usr/bin/env python3

import unittest
import json
import rospy

from ros_web_client.message_wrapper import Topic

from rosbridge_library.internal import pngcompression


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

    def test_decode_png(self):
        with open('pngmsg.txt', 'r') as outfile:
            data = outfile.read()  

        #dict_png=json.loads(sample_png)
        #data= dict_png["data"]

        #data="Salam1234!--- 'ASS':'12243.384764'"
        #data = pngcompression.encode(data)
        
        decoded = pngcompression.decode(data)

        with open('decodedagain_png.txt', 'w') as outfile:
            outfile.write(decoded) 
        
        print("done")

    def test_encode_png(self):
        with open('testDepth.txt', 'r') as outfile:
            data = outfile.read()  



        #data="Salam1234!--- 'ASS':'12243.384764'"
        #data = pngcompression.encode(data)
        encoded = pngcompression.encode(data)

        with open('encoded_png.txt', 'w') as outfile:
            outfile.write(encoded) 
        
        print("done")



PKG = 'ros_web_client'
NAME = 'test_initialization'
if __name__ == '__main__':
    rospy.init_node("test_rosbridge", anonymous=False)

    test_case = TestInitialization()
    #test_case.test_encode_png()
    test_case.test_decode_png()
    
    
    while not rospy.is_shutdown():
        rospy.spin()
    
    #import rosunit
    #rosunit.unitrun(PKG, NAME, TestInitialization)
