#!/usr/bin/env python3

import rospy
import json

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks, returnValue
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner
import txaio

from ros_web_client.protocol import ClientProtocol


#import umsgpack

class ClientSession(ApplicationSession):
    """
    """

    
    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.client_protocol = ClientProtocol(self)


#    def onConnect(self):          
#        self.join(self.config.realm)

    @inlineCallbacks
    def onJoin(self, details):
        rospy.loginfo("session attached")
        yield self.subscribe(self.on_data, client_params["data_domain"])
        yield self.register(self.on_command, client_params["service_domain"])


    @inlineCallbacks
    def on_command(self, command):
        result = yield self.client_protocol.incomingRPC(command)
        returnValue(result)

    def on_data(self,message):
        self.client_protocol.incoming_data(message)

    def publish_data(self,message):
        self.publish(server_params["data_domain"], message)

    @inlineCallbacks
    def call_RP(self,message):
        result = yield self.call(server_params["service_domain"], message)
        returnValue(result)
    



if __name__ == "__main__":
    rospy.init_node("clientside", anonymous=False) 
    #txaio.start_logging(level='debug')
    #Fetch parameters
    server_params = rospy.get_param("webclient/server/")
    client_params = rospy.get_param("webclient/client/") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)

