#!/usr/bin/env python3

import unittest
import json
import rospy

from sensor_msgs.msg import Image,CompressedImage
from tf2_msgs.msg import TFMessage


class Test(unittest.TestCase):
    
    def test_sync(self):
        with open('compressedDepth_sample_msg.txt', 'r') as outfile:
            sample_depth = outfile.read()  

        with open('compressedRGB_sample_msg.txt', 'r') as outfile:
            sample_rgb = outfile.read()  

        from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
        test_protocol=RosbridgeProtocol(88)

        test_protocol.incoming(sample_depth)
        test_protocol.incoming(sample_rgb)





if __name__ == '__main__':
    rospy.init_node("test", anonymous=False)

    test_case = Test()

    def callback_depth(data):
        print("------------Depth Header--------------")
        print(data.header)




    def callback_rgb(data):
        print("=============RGB Header==============")
        print(rospy.get_rostime().secs)
        print(data.header)

    def callback_tf(data):
        print("=============TF Header==============")
        print(data.transforms[0].header)



    rospy.Subscriber("/myimage/compressedDepth", CompressedImage, callback_depth)
    rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback_rgb)
    #rospy.Subscriber("/tf", TFMessage, callback_tf)

    test_case.test_sync()

    



    
    
    while not rospy.is_shutdown():
        print(rospy.Time.now().secs)
        rospy.spin()
    
