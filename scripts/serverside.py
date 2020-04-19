#!/usr/bin/env python3

import rospy
import json

from twisted.internet.defer import inlineCallbacks, returnValue
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from ros_web_client.initialization import ConfigHandler


class ClientSession(ApplicationSession):
    """
    """

    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.init()

    def init(self):
        self.ros_protocol = RosbridgeProtocol(0)
        self.ros_protocol.outgoing = self.outgoing

    @inlineCallbacks
    def onJoin(self, details):
        rospy.loginfo("session attached")

        yield self.subscribe(self.on_data, server_params["data_domain"])

        #Initialize Config
        config_handler = ConfigHandler(server_params["init_config"],self.ros_protocol)
        commands_list = config_handler.return_commands()
        for command in commands_list:
            result = yield self.call(client_params["service_domain"],command.message)
            command.callback(command,result)

    def on_data(self,message):
        self.ros_protocol.incoming(message)


    def outgoing(self, message):
        print("ROSBridge Outgoing {}".format(message))


if __name__ == "__main__":
    rospy.init_node("serverside", anonymous=False) 

    #Fetch parameters
    server_params = rospy.get_param("webclient/server/")
    client_params = rospy.get_param("webclient/client/") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)

