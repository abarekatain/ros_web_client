#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2020 Alireza Barekatain - Isfahan University of Technology
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import rospy
import json

from twisted.internet.defer import inlineCallbacks, returnValue
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner
import txaio

from ros_web_client.protocol import ServerProtocol


class ClientSession(ApplicationSession):
    """ Inherited Object from Crossbar ApplicationSession object to connect to WAMP router
    Called by Crossbar Application Runner
    """

    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.server_protocol = ServerProtocol(self)

    @inlineCallbacks
    def onJoin(self, details):
        """ See base class
        """
        rospy.loginfo("session attached")
        yield self.subscribe(self.on_data, server_params["data_domain"])
        yield self.register(self.on_command, server_params["service_domain"])

        self.server_protocol.initialize_config(server_params["init_config"])


    @inlineCallbacks
    def call_RP(self,message):
        """ RPC to client

            Args:
            message: command argument to pass to callee

            Yields:
            yield a differed and gives back control to reactor
            wait for the RPC response and return the result
        """
        result = yield self.call(client_params["service_domain"], message)
        returnValue(result)

    def on_data(self,message):
        """ Data Domain Subscriber -- Listen for incoming rosbridge messages 
        
            Args:
            message: rosbridge message string received
        """
        self.server_protocol.incoming_data(message)

    @inlineCallbacks
    def on_command(self, command):
        """ RPC Listener

            Args:
            command: string argument passed from caller

            Yields:
            yields a deffered from protocol RPC handler and gives back control until the result
            is ready, then returns the result to the caller
        """
        result = yield self.server_protocol.incomingRPC(command)
        returnValue(result)
    
    def publish_data(self,message):
        """ Publish to client data domain topic 

            Args:
            message: rosbridge message string to send
        """
        self.publish(client_params["data_domain"], message)



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

