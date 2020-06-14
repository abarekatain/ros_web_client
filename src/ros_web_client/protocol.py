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

import json

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks, returnValue

from ros_web_client.rosbridge_protocol_wrapper import ros_protocol as RosbridgeProtocol
from ros_web_client.services import ServiceHandler
from ros_web_client.initialization import ConfigHandler

class Protocol():
    """ Network-Level Protocol Base Class
        This class is the intermediary layer between WAMP Session and Rosbridge Protocol wrapper

        Attributes:
        session : Reference to the WAMP session class

        Methods:
        incoming_data
    """

    def __init__(self,session):
        self.session = session
        self.ros_protocol = RosbridgeProtocol(0)
        self.ros_protocol.outgoing = self.outgoing
        #Hold a list of topics and their corresponding thread's occupancy status
        self.topic_occupied = {}
        self.service_handler = ServiceHandler(self.ros_protocol)

    def incoming_data(self,message):
        """ Process Incoming Data from WAMP Data topic

            Args:
            message : incoming rosbridge message
        """
        self.ros_protocol.incoming(message)

    # the rosbridge outgoing
    def outgoing(self,message,isBinary=False,identifier=None):
        """ Overriden method of rosbridge protocol wrapper's Outgoing
            Guide each message to its corresponding sender

            Args:
            message : rosbridge protocol wrapper's outgoing message
            isBinary : If true, the message is of `publish` type and is binary serialized
            identifier : If isBinary=True , topic name will be aligned as identifier to
                         manage thread occupancy status
        """
        if isBinary:
            if not self.topic_occupied.get(identifier,False):
                reactor.callFromThread(self.publish_msg, message, identifier)
        else:
            message_dict = json.loads(message)
            op = message_dict.get("op")

            if op=="service_response":       
                self.service_handler.callback(message)

            elif op=="call_service":
                self.call_service(message)
            
            elif op=="publish":   
                thread_identifier = message_dict["topic"]
                if not self.topic_occupied.get(thread_identifier,False):   
                    reactor.callFromThread(self.publish_msg, message, thread_identifier)
                    
    @inlineCallbacks
    def call_service(self,message):
        """ Call WAMP session RPC caller

            Args:
            message : argument to pass to callee
        """
        result = yield self.session.call_RP(message)
        self.ros_protocol.incoming(result)

    def publish_msg(self,message,thread_identifier):
        """ Publish message on WAMP data topic
            This method is called from a thread. For every unique topic a unique thread
            is assigned to publish its message. This avoids occupying multiple threads 
            for a single topic.

            Args:
            message : data to publish
            thread_identifier : unique identifier (here topic name) to manage thread
            occupancy status
        """
        self.topic_occupied[thread_identifier]=True
        self.session.publish_data(message)
        self.topic_occupied[thread_identifier]=False


class ClientProtocol(Protocol):
    """ Network-Level Protocol for Clients -- See Base Class

        Methods:
        incomingRPC
    """

    def __init__(self,session):
        Protocol.__init__(self, session)

    @inlineCallbacks
    def incomingRPC(self,command):
        """ Manage Incoming RPC commands from server

            Args:
            command : The received command to process

            Yields:
            The result of processing the command
        """
        command_dict = json.loads(command)
        op = command_dict.get("op")

        if op=="call_service":
            result = yield self.service_handler.execute(command)
            returnValue(result)
        elif op=="request_service":
            #TODO Implement Allowed Services List
            returnValue(None)
        else:
            if op=="subscribe":
                # isIncoming=False declares that the topic messages are generated
                # on this machine
                self.ros_protocol.initialize_topic(message=command_dict,isIncoming=False)
            elif op=="advertise":
                # isIncoming=True declares that the topic messages are to be received
                # from external source
                self.ros_protocol.initialize_topic(message=command_dict,isIncoming=True)
            elif op=="advertise_service":
                self.ros_protocol.initialize_service(message=command_dict)
                
            self.ros_protocol.incoming(command_dict)
            returnValue(None)

class ServerProtocol(Protocol):
    """ Network-Level Protocol for Servers -- See Base Class

        Methods:
        initialize_config
        incomingRPC
    """

    @inlineCallbacks
    def initialize_config(self,configfile):
        """ Initialize Config File

            Args:
            configfile : content of initialization config
        """
        config_handler = ConfigHandler(configfile,self.ros_protocol)
        commands_list = config_handler.return_commands()
        for command in commands_list:
            result = yield self.session.call_RP(command.message)
            command.callback(command,result)

    @inlineCallbacks
    def incomingRPC(self,command):
        """ Manage Incoming RPC commands from client

            Args:
            command : The received command to process

            Yields:
            The result of processing the command
        """
        command_dict = json.loads(command)
        op = command_dict.get("op")
        if op=="call_service":
            result = yield self.service_handler.execute(command)
            returnValue(result)
