#!/usr/bin/env python3


import rospy
import json

from twisted.internet.defer import inlineCallbacks, returnValue
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner
import txaio

from ros_web_client.protocol import ServerProtocol


class ClientSession(ApplicationSession):
    """
    """

    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.server_protocol = ServerProtocol(self)

    @inlineCallbacks
    def onJoin(self, details):
        rospy.loginfo("session attached")
        yield self.subscribe(self.on_data, server_params["data_domain"])
        self.server_protocol.initialize_config(server_params["init_config"])


    @inlineCallbacks
    def call_client(self,message):
        result = yield self.call(client_params["service_domain"], message)
        returnValue(result)

    def on_data(self,message):
        self.server_protocol.incoming_data(message)



if __name__ == "__main__":
    rospy.init_node("serverside", anonymous=False) 
    #txaio.start_logging(level='debug')

    #Fetch parameters
    server_params = rospy.get_param("webclient/server/")
    client_params = rospy.get_param("webclient/client/") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)

