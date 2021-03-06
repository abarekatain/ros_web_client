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
import uuid


class Wrapper(object):
    """ Base class for all message-wrapper classes
    """
    SUPPORTED_COMPRESSION_TYPES = ('png', 'none')
    SUPPORTED_SERIALIZATION_TYPES = ('msgpack', 'json')

class Topic(Wrapper):
    """ Message Wrapper for Topics.

        Attributes:
        name : Topic name, e.g. ``/cmd_vel``.
        message_type : Message type, e.g. ``std_msgs/String``.
        compression : Type of compression to use, e.g. `png`. Defaults to `None`.
        serialization : Type of Serialization to use for transmitting messages
        throttle_rate : Rate (in ms between messages) at which to throttle the topics.
        queue_size : Queue size for re-publishing webtopics.
        latch : True to latch the topic when publishing, False otherwise.
        queue_length : Queue length used when subscribing.
        remap : If specified, The topic will be remapped to the name assigned to this argument
        parameters : Dictionary to pass all the arguments above at once
                     (Only one method of passing arguments should be used,
                      the value in `parameters` dict will be used otherwise)
    """
    

    def __init__(self, name, message_type, compression=None, serialization=None, latch=True, throttle_rate=0,
                 queue_size=100, queue_length=0,remap=None, parameters= {}):       
        self.name = name
        self.message_type = message_type
        self.compression = parameters.get("compression", compression)
        self.serialization = parameters.get("serialization", serialization) 
        self.latch = parameters.get("latch", latch) 
        self.throttle_rate = parameters.get("throttle_rate", throttle_rate) 
        self.queue_size = parameters.get("queue_size", queue_size)  
        self.queue_length = parameters.get("queue_length", queue_length)
        self.remap = parameters.get("remap", remap)

        self.op_id = uuid.uuid4().hex


        if self.compression is None:
            self.compression = 'none'
        
        if self.serialization is None:
            self.serialization = 'json'

        if self.remap is None:
            self.remap = self.name

        if self.compression not in self.SUPPORTED_COMPRESSION_TYPES:
            raise ValueError(
                'Unsupported compression type. Must be one of: ' + str(self.SUPPORTED_COMPRESSION_TYPES))
        
        if self.serialization not in self.SUPPORTED_SERIALIZATION_TYPES:
            raise ValueError(
                'Unsupported serialization type. Must be one of: ' + str(self.SUPPORTED_SERIALIZATION_TYPES))


    def subscribe_command(self):
        """Return a json subscription command for the topic

        """

        command = {
            'op': 'subscribe',
            'id': self.op_id,
            'type': self.message_type,
            'topic': self.name,
            '_compression': self.compression,
            '_serialization': self.serialization,
            '_remap': self.remap,
            'throttle_rate': self.throttle_rate,
            'queue_length': self.queue_length
        }

        return json.dumps(command)

    def unsubscribe_command(self):
        """Return a json unsubscription command for the topic
        """

        command = {
            'op': 'unsubscribe',
            'id': self.op_id,
            'topic': self.name
        }

        return json.dumps(command)    

    def advertise_command(self):
        """Return a json advertise command for the topic
        """

        command = {
            'op': 'advertise',
            'id': self.op_id,
            'type': self.message_type,
            'topic': self.name,
            '_compression': self.compression,
            '_serialization': self.serialization,
            '_remap': self.remap,
            'latch': self.latch,
            'queue_size': self.queue_size
        }

        return json.dumps(command)

    def unadvertise_command(self):
        """Return a json unadvertise command for the topic
        """
        
        command = {
            'op': 'unadvertise',
            'id': self.op_id,
            'topic': self.name,
        }

        return json.dumps(command)



class Service(Wrapper):
    """Message Wrapper for ROS services


        Attributes:
        name : Service name, e.g. ``/add_two_ints``.
        type : Service Type
        remap : If specified, The service will be remapped to the name assigned to this argument
        op_id : Unique Id for Service Call. if not specified, a unique ip will be assigned automatically
        parameters : Dictionary to pass all the arguments above at once
                     (Only one method of passing arguments should be used,
                      the value in `parameters` dict will be used otherwise)
    """

    def __init__(self, name, type=None,remap=None, op_id=None, compression=None, parameters= {}):
        self.name = name
        self.type = type
        self.compression = parameters.get("compression", compression)
        self.remap = parameters.get("remap", remap)
        self.op_id = op_id
        
        if self.compression is None:
            self.compression = 'none'
        if self.op_id is None:
            self.op_id = uuid.uuid4().hex

        if self.remap is None:
            self.remap = self.name

        if self.compression not in self.SUPPORTED_COMPRESSION_TYPES:
            raise ValueError(
                'Unsupported compression type. Must be one of: ' + str(self.SUPPORTED_COMPRESSION_TYPES))



    def call_command(self, request):
        """ return a json call-service command

            Args:
            request: request dictionary
        """

        command = {
            'op': 'call_service',
            'id': self.op_id,
            'service': self.name,
            'args': request,
        }

        return json.dumps(command)


    def advertise_command(self):
        """ return a json advertise-service command
        """

        command = {
            'op': 'advertise_service',
            "type": self.type,
            'service': self.name,
            "_remap": self.remap
        }

        return json.dumps(command)

    def request_command(self):
        """ return a json request-service command
        """
        command = {
            'op': 'request_service',
            "type": self.type,
            'service': self.name
        }

        return json.dumps(command)



class Param(object):
    """ Message Wrapper for ROS Parameters

        Attributes:
        name : Parameter name, e.g. ``max_vel_x``.
    """

    def __init__(self, name):
        self.name = name
        self.op_id = uuid.uuid4().hex

    def get_command(self):
        """ Return getParam Service Request Command
        """
        client = Service('/rosapi/get_param',op_id=self.op_id)
        request = {'name': self.name}
        return client.call_command(request)


    def set_command(self, value):
        """ Return setParam Service Request Command
        """
        
        client = Service('/rosapi/set_param',op_id=self.op_id)
        request = {'name': self.name, 'value': json.dumps(value)}

        return client.call_command(request)

    def delete_command(self):
        """ Return deleteParam Service Request Command
        """
        
        client = Service('/rosapi/delete_param',op_id=self.op_id)
        request = {'name': self.name}

        return client.call_command(request)


        


        
