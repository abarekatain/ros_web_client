import json
import rosparam

from ros_web_client.message_wrapper import Topic, Param


class CommandHandler(object):

    def callback_topic(self,command,result):
        if result is None:
            command.protocol.initialize_topic(wrapper=command.wrapper)
            command.protocol.incoming(command.wrapper.advertise_command())

    def callback_param(self,command,result):
        if result is not None:
            result=json.loads(result)
            param_value = result["values"]["value"]
            rosparam.set_param(command.wrapper.name,param_value)

class ConfigHandler(object):



    def __init__(self, config,protocol):
        self.protocol = protocol
        self.parser = ConfigParser(config)
        self.command_handler = CommandHandler()
        self.commands_list = []

    def handle_topics(self):

        for topic in self.parser.topics_list:
            command = Command(topic.subscribe_command(),
                        self.command_handler.callback_topic,
                        wrapper=topic,protocol=self.protocol)

            self.commands_list.append(command)
    
    def handle_params(self):
        
        for param in self.parser.params_list:
            command = Command(param.get_command(),
                        self.command_handler.callback_param,
                        wrapper=param,protocol=self.protocol)
            
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








            
