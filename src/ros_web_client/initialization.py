import json
import rosparam

from ros_web_client.message_wrapper import Topic, Param


class ConfigHandler(object):

    def __init__(self, config,protocol):
        self.protocol = protocol
        self.parser = ConfigParser(config)
        self.commands_list = []

    def handle_topics(self):

        def callback(command,result):
            if result is None:
                command.protocol.incoming(command.wrapper.advertise_command())

        for topic in self.parser.topics_list:
            command = Command(topic.subscribe_command(),callback,wrapper=topic,protocol=self.protocol)
            self.commands_list.append(command)
    
    def handle_params(self):

        def callback(command,result):
            if result is not None:
                result=json.loads(result)
                param_value = result["values"]["value"]
                rosparam.set_param(command.wrapper.name,param_value)
        
        for param in self.parser.params_list:
            command = Command(param.get_command(),callback,wrapper=param,protocol=self.protocol)
            self.commands_list.append(command)
    
    def return_commands(self):
        self.handle_topics()
        self.handle_params()
        return self.commands_list

class ConfigParser(object):
    """ Parser for Initialization config file in JSON

    Args:
        config (:obj: 'str') content of config file in JSON format
        protocol: (:obj: 'RosBridgeProtocol') instance of RosBridgeProtocol        
    """

    def __init__(self, config):
        self.config = json.loads(config)
        self.topics_list = []
        self.params_list = []
        self.parse()
    
    def parse_topics(self):
        """ parse topics into Topic wrapper class
        """
        for item in self.config.get("topics"):
            topic = Topic(item["name"],item["message_type"],parameters=item)
            self.topics_list.append(topic)
    
    def parse_params(self):
        """ parse params into param wrapper class
        """              
        for item in self.config.get("parameters"):
            param = Param(item)
            self.params_list.append(param)
    
    def parse(self):
        self.parse_topics()
        self.parse_params()


class Command(object):
    """ Command object to wrap all necessary data for RPC Call

    Args:
        message (:obj:'str') the command message to send
        callback (method) callback function to handle response
        wrapper(:obj:'Wrapper') message wrapper object, Topic, Service, or Param
    """

    def __init__(self,message,callback,wrapper=None,protocol=None):
        self.message = message
        self.callback = callback
        self.wrapper = wrapper
        self.protocol = protocol



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
        







            
