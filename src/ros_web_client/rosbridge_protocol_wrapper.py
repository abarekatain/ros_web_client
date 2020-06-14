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
import rospy

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.internal import pngcompression

import umsgpack

class ros_protocol():
    """ Rosbridge protocol wrapper to pre-process incoming messages, post-process
        outgoing messages, and store the settings and configs of each topic and service

        Methods:
        initialize_service
        initialize_topic
        incoming
        outgoing
    """

    def __init__(self,id):
        self._protocol = RosbridgeProtocol(id) 
        self._protocol.outgoing = self.rosbridge_outgoing

        #Store whether to compress/decompress topics
        self.incoming_topics_compression_state = {}
        self.outgoing_topics_compression_state = {}

        #Store whether to serialize/deserialize topics
        self.topics_serialization_state = {}

        #Store whether to remap topics/services
        self.topics_remap_state = {}
        self.services_remap_state = {}
        self.inv_services_remap_state = {}

    def initialize_service(self,wrapper=None,message=None):
        """ Initialize ROS service settings

            Args:
            wrapper/message : Either one of these arguments must be assigned
                              as service information
        """
        if wrapper is not None:
            name = wrapper.name
            remap = wrapper.remap
        elif message is not None:
            name = message.get("service")
            remap = message.get("_remap")
        
        self.setup_service_remaps(name,remap)
    
    def setup_service_remaps(self,name,remap):
        self.services_remap_state[name] = remap
        self.inv_services_remap_state = {v: k for k, v in self.services_remap_state.items()}

    def remap_service(self,service):
        return self.services_remap_state.get(service)

    def inv_remap_service(self,service):
        """ Re-remap the service """
        return self.inv_services_remap_state.get(service)

    def initialize_topic(self,wrapper=None,message=None,isIncoming=None):
        """ Initialize ROS topic settings

            Args:
            wrapper/message : Either one of these arguments must be assigned
                              as topic information
            isIncoming : If true, the topic messages are to be received from external source,
                         so setup remap and store the compression state for decompressing
                         If False, the topic messages are generated on this machine,
                         so no remapping is required and the compression state is stored for compressing
        """
        if wrapper is not None:
            topic = wrapper.name
            compression = wrapper.compression
            serialization = wrapper.serialization
            remap = wrapper.remap       
        elif message is not None:
            topic = message["topic"]
            compression = message.get("_compression","none")  
            serialization = message.get("_serialization","json")
            remap = message.get("_remap")
        
        if isIncoming:
            self.setup_topic_remaps(topic,remap)
        
        self.update_compression(topic,compression,isIncoming)
        self.update_serialization(topic,serialization)
    
    def setup_topic_remaps(self,name,remap):
        self.topics_remap_state[name] = remap

    def remap_topic(self,topic):
        return self.topics_remap_state.get(topic)

    def update_compression(self,topic,value,isIncoming):
        """ Store the  topic's compression status

            Args:
            topic : topic name
            value : compression state
            isIncoming : If True, the compression state is stored for decompressing
                         If False, the compression state is stored for compressing
        """
        if isIncoming:
            if topic not in self.incoming_topics_compression_state:
                self.incoming_topics_compression_state[topic] = value
        else:
            if topic not in self.outgoing_topics_compression_state:
                self.outgoing_topics_compression_state[topic] = value
    
    def update_serialization(self,topic,value):
        if topic not in self.topics_serialization_state:
            self.topics_serialization_state[topic] = value

    def decompress(self,topic,data):
        if self.incoming_topics_compression_state.get(topic) == "png":
            data = pngcompression.decode(data)
            return json.loads(data)
        else:
            return data

    def compress(self,topic,data):
        if self.outgoing_topics_compression_state.get(topic) == "png":
            data = json.dumps(data)
            return pngcompression.encode(data)
        else: 
            return data 

    def serialize(self,topic,data):
        if self.topics_serialization_state[topic] == "msgpack":
            data = umsgpack.packb(data)
            return data
        else:
            return json.dumps(data) 
    
    def deserialize(self,data):
        data = umsgpack.unpackb(data)
        return data


    def incoming(self,message):
        """ Intermediary incoming method to pre-process the message

            Args:
            message : rosbridge message
        """
        #Convert to Dictionary, Whatever the input is
        if isinstance(message, str):
            message = json.loads(message)
        elif isinstance(message, bytes):
            message = self.deserialize(message)

        op = message.get("op")
        if op == "publish":
            message["msg"] = self.decompress(message["topic"],message.get("msg"))
            message["topic"] = self.remap_topic(message["topic"])       
        elif op == "advertise":
            message["topic"] = self.remap_topic(message["topic"])
        elif op == "advertise_service" or op == "service_response":
            message["service"] = self.remap_service(message["service"])


        message = json.dumps(message)
        #--------
        #replace JSON Null values in float32 types with infinity datatype (changed according to the error for LaserScan values)
        message = message.replace("null", "Infinity")
        #--------
        self._protocol.incoming(message)

    def rosbridge_outgoing(self,message):
        """ Overriden method of rosbridge protocol's Outgoing
        """

        message = json.loads(message)

        op=message.get("op")
        if op == "publish":
            topic_name = message["topic"]
            message["msg"] = self.compress(topic_name,message.get("msg"))
            message = self.serialize(topic_name,message)
        elif op == "call_service":
            message["service"] = self.inv_remap_service(message["service"])

        
        if isinstance(message, bytes):
            self.outgoing(message,isBinary=True,identifier=topic_name)
        elif isinstance(message,str):
            self.outgoing(message)
        else:
            message = json.dumps(message)
            self.outgoing(message)

    def outgoing(self,message,isBinary=False,identifier=None):
        """ Pass an outgoing message.  This method should be overridden.

            Args:
            message : rosbridge protocol wrapper's outgoing message
            isBinary : If true, the message is of `publish` type and is binary serialized
            identifier : If isBinary=True , topic name will be aligned as identifier to
                         manage thread occupancy status
        """
        pass