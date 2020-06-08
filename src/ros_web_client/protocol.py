
import json

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks, returnValue

from ros_web_client.rosbridge_protocol_wrapper import ros_protocol as RosbridgeProtocol
from ros_web_client.services import ServiceHandler
from ros_web_client.initialization import ConfigHandler

class Protocol():

    def __init__(self,session):
        self.session = session
        self.ros_protocol = RosbridgeProtocol(0)
        self.ros_protocol.outgoing = self.outgoing
        self.topic_occupied = {}
        self.service_handler = ServiceHandler(self.ros_protocol)

    def incoming_data(self,message):
        self.ros_protocol.incoming(message)

    # the rosbridge outgoing
    def outgoing(self,message,isBinary=False,identifier=None):
        if isBinary:
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
        result = yield self.session.call_RP(message)
        self.ros_protocol.incoming(result)

    def publish_msg(self,message,thread_identifier):
        self.topic_occupied[thread_identifier]=True
        self.session.publish_data(message)
        self.topic_occupied[thread_identifier]=False


class ClientProtocol(Protocol):

    def __init__(self,session):
        Protocol.__init__(self, session)

    @inlineCallbacks
    def incomingRPC(self,command):
        command_dict = json.loads(command)
        op = command_dict.get("op")
        if op=="call_service":
            result = yield self.service_handler.execute(command)
            returnValue(result)
        elif op=="request_service":
            returnValue(None)
        else:
            if op=="subscribe":
                self.ros_protocol.initialize_topic(message=command_dict,isIncoming=False)
            elif op=="advertise":
                self.ros_protocol.initialize_topic(message=command_dict,isIncoming=True)

            self.ros_protocol.incoming(command_dict)
            returnValue(None)

class ServerProtocol(Protocol):

    @inlineCallbacks
    def initialize_config(self,configfile):
        config_handler = ConfigHandler(configfile,self.ros_protocol)
        commands_list = config_handler.return_commands()
        for command in commands_list:
            result = yield self.session.call_RP(command.message)
            command.callback(command,result)

    @inlineCallbacks
    def incomingRPC(self,command):
        command_dict = json.loads(command)
        op = command_dict.get("op")
        if op=="call_service":
            result = yield self.service_handler.execute(command)
            returnValue(result)
