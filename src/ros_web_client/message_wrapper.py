from __future__ import print_function

import json
import uuid

# Python 2/3 compatibility import list
try:
    from collections import UserDict
except ImportError:
    from UserDict import UserDict

class Wrapper(object):
    SUPPORTED_COMPRESSION_TYPES = ('png', 'none')
    SUPPORTED_SERIALIZATION_TYPES = ('msgpack', 'json')

class Topic(Wrapper):
    """message wrapper for topics.

    Args:
        name (:obj:`str`): Topic name, e.g. ``/cmd_vel``.
        message_type (:obj:`str`): Message type, e.g. ``std_msgs/String``.
        compression (:obj:`str`): Type of compression to use, e.g. `png`. Defaults to `None`.
        throttle_rate (:obj:`int`): Rate (in ms between messages) at which to throttle the topics.
        queue_size (:obj:`int`): Queue size created at bridge side for re-publishing webtopics.
        latch (:obj:`bool`): True to latch the topic when publishing, False otherwise.
        queue_length (:obj:`int`): Queue length at bridge side used when subscribing.
        reconnect_on_close (:obj:`bool`): Reconnect the topic (both for publisher and subscribers) if a reconnection is detected.
    """
    

    def __init__(self, name, message_type, compression=None, serialization=None, latch=True, throttle_rate=0,
                 queue_size=100, queue_length=0, parameters= {}):       
        self.name = name
        self.message_type = message_type
        self.compression = parameters.get("compression", compression)
        self.serialization = parameters.get("serialization", serialization) 
        self.latch = parameters.get("latch", latch) 
        self.throttle_rate = parameters.get("throttle_rate", throttle_rate) 
        self.queue_size = parameters.get("queue_size", queue_size)  
        self.queue_length = parameters.get("queue_length", queue_length)

        self.op_id = uuid.uuid4().hex


        if self.compression is None:
            self.compression = 'none'
        
        if self.serialization is None:
            self.serialization = 'json'

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
            'throttle_rate': self.throttle_rate,
            'queue_length': self.queue_length
        }

        return json.dumps(command)

    def unsubscribe_command(self):
        """Return a json unsubscription command for the topic"""
        command = {
            'op': 'unsubscribe',
            'id': self.op_id,
            'topic': self.name
        }

        return json.dumps(command)    

    def advertise_command(self):
        """Return a json advertise command for the topic"""

        command = {
            'op': 'advertise',
            'id': self.op_id,
            'type': self.message_type,
            'topic': self.name,
            'latch': self.latch,
            'queue_size': self.queue_size
        }

        return json.dumps(command)

    def unadvertise_command(self):
        """Return a json unadvertise command for the topic"""
        
        command = {
            'op': 'unadvertise',
            'id': self.op_id,
            'topic': self.name,
        }

        return json.dumps(command)







class Service(Wrapper):
    """message wrapper for ROS services.


    Args:
        name (:obj:`str`): Service name, e.g. ``/add_two_ints``.
    """

    def __init__(self, name, op_id=None, compression=None):
        self.name = name
        self.compression = compression
        self.op_id = op_id
        
        if self.compression is None:
            self.compression = 'none'
        if self.op_id is None:
            self.op_id = uuid.uuid4().hex

        if self.compression not in self.SUPPORTED_COMPRESSION_TYPES:
            raise ValueError(
                'Unsupported compression type. Must be one of: ' + str(self.SUPPORTED_COMPRESSION_TYPES))



    def call_command(self, request):


        command = {
            'op': 'call_service',
            'id': self.op_id,
            'service': self.name,
            'args': request,
        }

        return json.dumps(command)






class Param(object):
    """A ROS parameter.

    Args:
        ros (:class:`.Ros`): Instance of the ROS connection.
        name (:obj:`str`): Parameter name, e.g. ``max_vel_x``.
    """

    def __init__(self, name):
        self.name = name
        self.op_id = uuid.uuid4().hex

    def get_command(self):
        client = Service('/rosapi/get_param',op_id=self.op_id)
        request = {'name': self.name}
        return client.call_command(request)


    def set_command(self, value):

        client = Service('/rosapi/set_param',op_id=self.op_id)
        request = {'name': self.name, 'value': json.dumps(value)}

        return client.call_command(request)

    def delete_command(self):
        
        client = Service('/rosapi/delete_param',op_id=self.op_id)
        request = {'name': self.name}

        return client.call_command(request)


        


        
