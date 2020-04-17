#!/usr/bin/env python3

import rospy
import json

from twisted.internet.defer import inlineCallbacks, returnValue
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from ros_web_client.services import ServiceHandler


class ClientSession(ApplicationSession):
    """
    """

    
    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.init()

    def init(self):
        self.ros_protocol = RosbridgeProtocol(0)
        self.service_handler = ServiceHandler(self.ros_protocol)   
        self.ros_protocol.outgoing = self.outgoing

    @inlineCallbacks
    def onJoin(self, details):
        rospy.loginfo("session attached")
        yield self.register(self.on_command, client_params["service_domain"])


    @inlineCallbacks
    def on_command(self, command):
        command_dict = json.loads(command)
        if command_dict.get("op")=="call_service":
            result = yield self.service_handler.execute(command)
            returnValue(result)
        else:
            self.ros_protocol.incoming(command)
            returnValue(None)

    def outgoing(self, message):
        print("ROSBridge Outgoing {}".format(message))
        self.service_handler.callback(message)


if __name__ == "__main__":
    rospy.init_node("clientside", anonymous=False) 

    #Fetch parameters
    client_params = rospy.get_param("webclient/client/") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)

