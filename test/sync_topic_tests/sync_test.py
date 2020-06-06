#!/usr/bin/env python3

import unittest
import json
import rospy

from sensor_msgs.msg import Image,CompressedImage
from tf2_msgs.msg import TFMessage


class Test(unittest.TestCase):
    pass





if __name__ == '__main__':
    rospy.init_node("test", anonymous=False)

    test_case = Test()

    def callback_depth(data):
        print("------------Depth Header--------------")
        print(data.header)



    def callback_rgb(data):
        print("=============RGB Header==============")
        print(data.header)

    def callback_tf(data):
        print("=============TF Header==============")
        print(data.transforms[0].header)



    rospy.Subscriber("/webclient/myimage/image_raw", Image, callback_depth)
    rospy.Subscriber("/webclient/camera/rgb/image_raw", Image, callback_rgb)
    #rospy.Subscriber("/tf", TFMessage, callback_tf)



    
    
    while not rospy.is_shutdown():
        rospy.spin()
    
