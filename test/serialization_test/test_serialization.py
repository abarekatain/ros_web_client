#!/usr/bin/env python3

import unittest
import json
import rospy

import umsgpack

from sensor_msgs.msg import CompressedImage


class Test(unittest.TestCase):

    def test_serialize_deserialize(self):
        with open('sample_scan.txt', 'r') as outfile:
            data = outfile.read()  

        data = json.loads(data)

        msgpacked = umsgpack.packb(data)

        with open("encoded_msg.txt", "wb") as outfile:
            outfile.write(msgpacked)

        decoded = umsgpack.unpackb(msgpacked)

        with open('decodedagain_png.txt', 'w') as outfile:
            outfile.write(json.dumps(decoded)) 
        
        print("done")



PKG = 'ros_web_client'
NAME = 'test_initialization'
if __name__ == '__main__':
    rospy.init_node("test_rosbridge", anonymous=False)

    #test_case = Test()
    #test_case.test_serialize_deserialize()

    def callback_compressed(data):
        with open('sample_scan.txt', 'r') as outfile:
            data = outfile.read()  

    
    rospy.Subscriber('/camera/rgb/image_raw/compressed',CompressedImage, callback_compressed)


    
    
    while not rospy.is_shutdown():
        rospy.spin()
    
