import json

from ros_web_client.message_wrapper import Topic, Param

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol


class ConfigParser(object):
    """ Parser for Initialization config file in JSON

    Args:
        config (:obj: 'str') content of config file in JSON format
    """

    def __init__(self, config):
        self.config = json.loads(config)
        self.list_topics = []
        self.list_params = []
        self.list_commands = []
    
    def parse_topics(self):
        for item in self.config.get("topics"):
            topic=Topic(item["name"],item["message_type"],parameters=item)
            self.list_topics.append(topic)
            self.list_commands.append(topic.subscribe_command())
    
    def parse_params(self):
        for item in self.config.get("parameters"):
            param= Param(item)
            self.list_params.append(param)
            self.list_commands.append(param.get_command())
    
    def parse(self):
        self.parse_topics()
        self.parse_params()
        return self.list_commands

class CommandExecuter(object):

    def __init__(self):
        self.protocol = RosbridgeProtocol(0)
        self.protocol.outgoing = self.outgoing
        self.list_responses = {}

    def outgoing(self, message):
        pass    
    
    def execute(self, list_commands):
        for command in list_commands:
            self.protocol.incoming(command)





            
