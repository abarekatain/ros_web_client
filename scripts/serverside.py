#!/usr/bin/env python3

import rospy
import json

from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from ros_web_client.initialization import ConfigParser


class ClientSession(ApplicationSession):
    """
    """

    
    def __init__(self, config):
        ApplicationSession.__init__(self, config)
        self.init()

    def init(self):
        pass   

    @inlineCallbacks
    def onJoin(self, details):
        rospy.loginfo("session attached")

        #TEMP----------------------
        
        #Register command procedure
        yield self.register(self.on_command, client_params["service_domain"])

        #TEMP----------------------

        config_parser=ConfigParser(server_params["init_config"])
        list_commands=config_parser.parse()

        for command in list_commands:
            result = yield self.call(client_params["service_domain"],command)
            if result is not None:
                self.process_service_result(result)

    def process_service_result(self,result):
        #TODO currently only the parameter processing is required, Add more processes later
        dict_result = json.loads(result)

    def on_data(self,message):
        pass


    def on_command(self, command):
        pass


    def outgoing(self, message):
        pass


if __name__ == "__main__":
    rospy.init_node("serverside", anonymous=False) 

    #Fetch parameters
    server_params = rospy.get_param("webclient/server/")
    client_params = rospy.get_param("webclient/client/") 
    url = rospy.get_param("webclient/url") 
    realm = rospy.get_param("webclient/realm") 


    runner = ApplicationRunner(url=url, realm=realm)
    runner.run(ClientSession, auto_reconnect=True)

