import json

from ros_web_client.message_wrapper import Topic, Param


from twisted.internet import defer


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

class ServiceHandler(object):
    """ Handle Service Requests from RPC
        catch the request and return a deffered,
        the deffered will callback when request is completed

        Args:
        protocol (:obj: ROSBridgeProtocol) instance of Rosbridge Protocol
    """

    def __init__(self,protocol):
        self.protocol = protocol
        self.service_deffereds={}
   
    
    def execute(self, command):
        dict_command=json.loads(command)
        d=defer.Deferred()
        d.addCallback(self._return_result)
        self.service_deffereds[dict_command["id"]]=d
        self.protocol.incoming(command)
        return d

    def callback(self,message):
        dict_message=json.loads(message)
        d=self.service_deffereds.get(dict_message["id"])
        d.callback(message)
        del self.service_deffereds[dict_message["id"]]

    def _return_result(self,message):
        return message
        







            
