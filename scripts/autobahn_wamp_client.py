#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from ros_web_client.message_wrapper import Topic

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner


class ClientSession(ApplicationSession):
    """
    """

    @inlineCallbacks
    def onJoin(self, details):
        print("session attached")

        #Register command procedure
        def on_command(command):
            print("recieved command is {}".format(command))

        try:
            yield self.register(on_command, command_domain)
        except Exception as e:
            print("failed to register procedure: {}".format(e))
        else:
            print("command procedure registered")


        #Call command procedure
        """
        try:
            topic= Topic(1,"dadatopic","std_msgs/String")
            now = yield self.call(command_domain,topic.subscribe_command())
        except Exception as e:
            print("Error: {}".format(e))
        else:
            print("command sent")

        """      

    def onDisconnect(self):
        print("disconnected")



if __name__ == "__main__":
    rospy.init_node("autobahn_wamp_client", anonymous=False)

    #Fetch parameters
    data_domain = rospy.get_param("webclient/data_domain")
    command_domain = rospy.get_param("webclient/command_domain") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)


    print("main gone")