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

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from ros_web_client.services import ServiceHandler


def _log_exception():
    """Log the most recent exception to ROS."""
    exc = traceback.format_exception(*sys.exc_info())
    rospy.logerr(''.join(exc))


def log_exceptions(f):
    """Decorator for logging exceptions to ROS."""
    @wraps(f)
    def wrapper(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except:
            _log_exception()
            raise
    return wrapper



@implementer(interfaces.IPushProducer)
class OutgoingValve:
    """Allows the Autobahn transport to pause outgoing messages from rosbridge.
    
    The purpose of this valve is to connect backpressure from the WebSocket client
    back to the rosbridge protocol, which depends on backpressure for queueing.
    Without this flow control, rosbridge will happily keep writing messages to
    the WebSocket until the system runs out of memory.

    This valve is closed and opened automatically by the Twisted TCP server.
    In practice, Twisted should only close the valve when its userspace write buffer
    is full and it should only open the valve when that buffer is empty.

    When the valve is closed, the rosbridge protocol instance's outgoing writes
    must block until the valve is opened.
    """
    def __init__(self, session):
        self._session = session
        self._valve = threading.Event()
        self._finished = False

    @log_exceptions
    def relay(self, message):
        self._valve.wait()
        if self._finished:
            return
        #reactor.callFromThread(self._session.outgoing, message)
        self._session.publish_msg(message)

    def pauseProducing(self):
        if not self._finished:
            self._valve.clear()

    def resumeProducing(self):
        self._valve.set()

    def stopProducing(self):
        self._finished = True
        self._valve.set()



class ClientSession(ApplicationSession):
    """
    """

    
    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.init()

    def init(self):
        self.ros_protocol = RosbridgeProtocol(0)
        self.producer = OutgoingValve(self)
        self.ros_protocol.outgoing = self.outgoing   
        self.service_handler = ServiceHandler(self.ros_protocol)

    def onConnect(self):          

        self._transport.registerProducer(self.producer, True)
        self.producer.resumeProducing()
        self.join(self.config.realm)

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

    def outgoing(self, message):
        print("ROSBridge Outgoing {}".format(message))
        message_dict = json.loads(message)
        if message_dict.get("op")=="service_response":         
            self.service_handler.callback(message)
        else:
            self.producer.relay(message)

    def publish_msg(self,message):
        self.publish(server_params["data_domain"], message)


if __name__ == "__main__":
    rospy.init_node("clientside", anonymous=False) 

    #Fetch parameters
    server_params = rospy.get_param("webclient/server/")
    client_params = rospy.get_param("webclient/client/") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)

