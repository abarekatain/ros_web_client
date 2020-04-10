#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from ros_web_client.message_wrapper import Topic

import sys
import threading
import traceback
from functools import wraps

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner
from twisted.internet import interfaces, reactor
from zope.interface import implementer

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol



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
        self._session.outgoing(message)

    def pauseProducing(self):
        #print("-------------------------------PAUSED")
        if not self._finished:
            self._valve.clear()
        #rospy.signal_shutdown("DADADADADADADADAD")

    def resumeProducing(self):
        #print("-------------------------------RESUMED")
        self._valve.set()

    def stopProducing(self):
        self._finished = True
        self._valve.set()
        #rospy.signal_shutdown("DADADADADADADADAD")







class ClientSession(ApplicationSession):
    """
    """

    
    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.init()

    def init(self):
        self.ros_protocol = RosbridgeProtocol(0)
        self.producer = OutgoingValve(self)
        self.ros_protocol.outgoing = self.producer.relay


    
    def onConnect(self):          

        self._transport.registerProducer(self.producer, True)
        self.producer.resumeProducing()

        self.join(self.config.realm)
    

    @inlineCallbacks
    def onJoin(self, details):
        print("session attached")

        #Subscribe
        #yield self.subscribe(self.on_data, data_domain)

        #Register command procedure
        try:
            yield self.register(self.on_command, command_domain)
        except Exception as e:
            print("failed to register procedure: {}".format(e))
        else:
            print("command procedure registered")

        #Fake: Call command procedure
        '''
        topic= Topic(1,"dadatopic","std_msgs/String")
        self.ros_protocol.incoming(topic.advertise_command())
        try:
            yield self.call(command_domain,topic.subscribe_command())
        except Exception as e:
            print("Error: {}".format(e))
        else:
            print("command sent")
        '''    
    

    def on_data(self,message):
        self.ros_protocol.incoming(command)


    def on_command(self, command):
        print("recieved command is {}".format(command))
        self.ros_protocol.incoming(command)


    def outgoing(self, message):
        self.publish(data_domain, message)
        #print("outgoing message from rosbridgeprotocole")
        

    def onDisconnect(self):
        print("disconnected")



if __name__ == "__main__":
    rospy.init_node("autobahn_wamp_client", anonymous=False) 
    ''', disable_signals=True'''

    #Fetch parameters
    data_domain = rospy.get_param("webclient/data_domain")
    command_domain = rospy.get_param("webclient/command_domain") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)


    print("main gone")