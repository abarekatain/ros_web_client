
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


class ClientProtocol(Protocol):

    def __init__(self,session):
        Protocol.__init__(self, session)
        self.service_handler = ServiceHandler(self.ros_protocol)
        self.topic_occupied = {}
   

    @inlineCallbacks
    def incomingRPC(self,command):
        command_dict = json.loads(command)
        if command_dict.get("op")=="call_service":
            result = yield self.service_handler.execute(command)
            returnValue(result)
        else:
            self.ros_protocol.incoming(command_dict)
            returnValue(None)

    # the rosbridge outgoing
    def outgoing(self, message):
        message_dict = json.loads(message)
        #msgpacked = umsgpack.packb(message_dict)

        if message_dict.get("op")=="service_response":       
            self.service_handler.callback(message)
        
        elif message_dict.get("op")=="publish":   
            thread_identifier = message_dict["topic"]
            if not self.topic_occupied.get(thread_identifier,False):   
                reactor.callFromThread(self.publish_msg, message, thread_identifier)

    def publish_msg(self,message,thread_identifier):
        self.topic_occupied[thread_identifier]=True
        self.session.publish_data(message)
        self.topic_occupied[thread_identifier]=False



class ServerProtocol(Protocol):

    def __init__(self,session):
        Protocol.__init__(self, session)

    @inlineCallbacks
    def initialize_config(self,configfile):
        config_handler = ConfigHandler(configfile,self.ros_protocol)
        commands_list = config_handler.return_commands()
        for command in commands_list:
            result = yield self.session.call_client(command.message)
            command.callback(command,result)

    def incoming_data(self,message):
        self.ros_protocol.incoming(message)

    def outgoing(self, message):
        print("ROSBridge Outgoing {}".format(message))    