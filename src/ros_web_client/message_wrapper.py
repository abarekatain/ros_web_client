from __future__ import print_function

import json

# Python 2/3 compatibility import list
try:
    from collections import UserDict
except ImportError:
    from UserDict import UserDict


class Topic(object):
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
    SUPPORTED_COMPRESSION_TYPES = ('png', 'none')

    def __init__(self, name, message_type, compression=None, latch=True, throttle_rate=0,
                 queue_size=100, queue_length=0, parameters= {}):       
        self.name = name
        self.message_type = message_type
        self.compression = parameters.get("compression", compression) 
        self.latch = parameters.get("latch", latch) 
        self.throttle_rate = parameters.get("throttle_rate", throttle_rate) 
        self.queue_size = parameters.get("queue_size", queue_size)  
        self.queue_length = parameters.get("queue_length", queue_length)

        self.subscribe_id = None
        self.advertise_id = None

        if self.compression is None:
            self.compression = 'none'

        if self.compression not in self.SUPPORTED_COMPRESSION_TYPES:
            raise ValueError(
                'Unsupported compression type. Must be one of: ' + str(self.SUPPORTED_COMPRESSION_TYPES))


    @property
    def is_advertised(self):
        """Indicate if the topic is currently advertised or not.

        Returns:
            bool: True if advertised as publisher of this topic, False otherwise.
        """
        return self.advertise_id is not None

    @property
    def is_subscribed(self):
        """Indicate if the topic is currently subscribed or not.

        Returns:
            bool: True if subscribed to this topic, False otherwise.
        """
        return self.subscribe_id is not None

    def subscribe_command(self):
        """Return a json subscription command for the topic

        """
        if not self.subscribe_id:
            self.subscribe_id = 'subscribe:%s' % (
                self.name)

        command = {
            'op': 'subscribe',
            'id': self.subscribe_id,
            'type': self.message_type,
            'topic': self.name,
            'compression': self.compression,
            'throttle_rate': self.throttle_rate,
            'queue_length': self.queue_length
        }

        return json.dumps(command)

    def unsubscribe_command(self):
        """Return a json unsubscription command for the topic"""
        command = {
            'op': 'unsubscribe',
            'topic': self.name
        }

        if self.subscribe_id:
            command['id'] = self.subscribe_id
            self.subscribe_id = None

        return json.dumps(command)    

    def advertise_command(self):
        """Return a json advertise command for the topic"""
        
        if not self.is_advertised:
            self.advertise_id = 'advertise:%s' % (
                self.name)

        command = {
            'op': 'advertise',
            'id': self.advertise_id,
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
            'topic': self.name,
        }

        if self.is_advertised:
            command['id']= self.advertise_id
            self.advertise_id = None

        return json.dumps(command)







class Service(object):
    """message wrapper for ROS services.


    Args:
        name (:obj:`str`): Service name, e.g. ``/add_two_ints``.
    """
    SUPPORTED_COMPRESSION_TYPES = ('png', 'none')

    def __init__(self, name, compression=None):
        self.name = name
        self.compression = compression
        
        if self.compression is None:
            self.compression = 'none'

        if self.compression not in self.SUPPORTED_COMPRESSION_TYPES:
            raise ValueError(
                'Unsupported compression type. Must be one of: ' + str(self.SUPPORTED_COMPRESSION_TYPES))



    def call_command(self, request):

        service_call_id = 'call_service:%s' % (
            self.name)

        command = {
            'op': 'call_service',
            'id': service_call_id,
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

    def get_command(self):
        client = Service('/rosapi/get_param')
        request = {'name': self.name}
        return client.call_command(request)


    def set_command(self, value):

        client = Service('/rosapi/set_param')
        request = {'name': self.name, 'value': json.dumps(value)}

        return client.call_command(request)

    def delete_command(self):
        
        client = Service('/rosapi/delete_param')
        request = {'name': self.name}

        return client.call_command(request)


        


        
