#!/usr/bin/env python3

# Copyright (c) 2020, Alireza Barekatain - Isfahan University of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


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
            message: rosbridge message received
        """
        self.server_protocol.incoming_data(message)

    @inlineCallbacks
    def on_command(self, command):
        """ RPC Listener

            Args:
            command: argument passed from caller

            Yields:
            yields a deffered from protocol RPC handler and gives back control until the result
            is ready, then returns the result to the caller
        """
        result = yield self.server_protocol.incomingRPC(command)
        returnValue(result)
    
    def publish_data(self,message):
        """ Publish to client data domain topic 

            Args:
            message: rosbridge message to send
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

