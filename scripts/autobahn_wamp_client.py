#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from ros_web_client.message_wrapper import Topic

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol


class ClientSession(ApplicationSession):
    """
    """

    @inlineCallbacks
    def onJoin(self, details):
        print("session attached")
        
        #Instantiate RosBridgeProtocol
        self.protocol = RosbridgeProtocol(0)
        self.protocol.outgoing = self.outgoing
     
        #Register command procedure
        try:
            yield self.register(self.on_command, command_domain)
        except Exception as e:
            print("failed to register procedure: {}".format(e))
        else:
            print("command procedure registered")

        #Call command procedure
        try:
            topic= Topic(1,"dadatopic","std_msgs/String")
            yield self.call(command_domain,topic.subscribe_command())
        except Exception as e:
            print("Error: {}".format(e))
        else:
            print("command sent")
    

    def on_command(self, command):
        print("recieved command is {}".format(command))
        self.protocol.incoming(command)


    def outgoing(self, message):
        print("outgoing message from rosbridgeprotocole is {}".format(message))
        

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