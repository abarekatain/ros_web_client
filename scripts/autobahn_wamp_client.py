#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from ros_web_client.message_wrapper import Topic

if __name__ == "__main__":
    rospy.init_node('autobahn_wamp_client', anonymous=False)

    topic= Topic(1,"dadatopic","std_msgs/String")

    rospy.loginfo(topic.unsubscribe_command())


    print("main gone")