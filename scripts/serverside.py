#!/usr/bin/env python3

import rospy
import json

from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from ros_web_client.initialization import ConfigParser


class ClientSession(ApplicationSession):
    """
    """

    
    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.init()

    def init(self):
        self.ros_protocol = RosbridgeProtocol(0)   

    @inlineCallbacks
    def onJoin(self, details):
        rospy.loginfo("session attached")

        #TEMP----------------------
        
        #Register command procedure
        yield self.register(self.on_command, client_params["service_domain"])

        #TEMP----------------------

        config_parser=ConfigParser(server_params["init_config"],self.ros_protocol)
        list_commands=config_parser.parse()

        for command in list_commands:
            result = yield self.call(client_params["service_domain"],command.message)
            command.callback(command,result)


    def on_data(self,message):
        pass


    def on_command(self, command):
        teststr='{"op":"service_response","service":"\/rosapi\/get_param","values":{"value":"88"},"result":true}'
        return teststr

    def outgoing(self, message):
        pass


if __name__ == "__main__":
    rospy.init_node("serverside", anonymous=False) 

    #Fetch parameters
    server_params = rospy.get_param("webclient/server/")
    client_params = rospy.get_param("webclient/client/") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)

