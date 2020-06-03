#!/usr/bin/env python3

import sys
import threading
import traceback
from functools import wraps

import rospy
import json

from twisted.internet import interfaces, reactor
from zope.interface import implementer
from twisted.internet.defer import inlineCallbacks, returnValue
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner
import txaio

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
        self.ros_protocol.outgoing = self.outgoing   
        self.service_handler = ServiceHandler(self.ros_protocol)

        self.topic_occupied = {}

#    def onConnect(self):          
#        self.join(self.config.realm)

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
    # the rosbridge Outgoing
    def outgoing(self, message):
        message_dict = json.loads(message)

        if message_dict.get("op")=="service_response":         
            self.service_handler.callback(message)
        elif message_dict.get("op")=="publish" and self.topic_occupied.get(message_dict["topic"],False)==False:
            reactor.callFromThread(self.publish_msg, message)


    def publish_msg(self,message):
        message_dict = json.loads(message)
        self.topic_occupied[message_dict["topic"]]=True
        self.publish(server_params["data_domain"], message)
        self.topic_occupied[message_dict["topic"]]=False



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

