import json
import rosparam

from ros_web_client.message_wrapper import Topic, Param


class CommandHandler(object):

    def callback_topic_client(self,command,result):
        if result is None:
            command.protocol.initialize_topic(wrapper=command.wrapper,isIncoming=True)
            command.protocol.incoming(command.wrapper.advertise_command())

    def callback_topic_server(self,command,result):
        if result is None:
            command.protocol.initialize_topic(wrapper=command.wrapper,isIncoming=False)
            command.protocol.incoming(command.wrapper.subscribe_command())

    def callback_param_client(self,command,result):
        if result is not None:
            result=json.loads(result)
            param_value = result["values"]["value"]
            rosparam.set_param(command.wrapper.name,param_value)

class ConfigHandler(object):

    def __init__(self,config,protocol):
        self.protocol = protocol
        self.parser = ConfigParser(config)
        self.command_handler = CommandHandler()
        self.commands_list = []

    def handle_topics(self):
        #Client
        for topic in self.parser.client_topics_list:
            command = Command(topic.subscribe_command(),
                        self.command_handler.callback_topic_client,
                        wrapper=topic,protocol=self.protocol)
            self.commands_list.append(command)

        #Server
        for topic in self.parser.server_topics_list:
            command = Command(topic.advertise_command(),
                        self.command_handler.callback_topic_server,
                        wrapper=topic,protocol=self.protocol)
            self.commands_list.append(command)
    
    def handle_params(self):
        #Client
        for param in self.parser.client_params_list:
            command = Command(param.get_command(),
                        self.command_handler.callback_param_client,
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
        config = json.loads(config)
        self.client_config = config.get("client")
        self.server_config = config.get("server")

        self.client_topics_list = []
        self.client_services_list = []
        self.client_params_list = []

        self.server_topics_list = []
        self.server_services_list = []

        self.parse()
    
    def parse_topics(self):
        """ parse topics into Topic wrapper class
        """
        #Client
        for item in self.client_config.get("topics"):
            topic = Topic(item["name"],item["message_type"],parameters=item)
            self.client_topics_list.append(topic)

        #Server
        for item in self.server_config.get("topics"):
            topic = Topic(item["name"],item["message_type"],parameters=item)
            self.server_topics_list.append(topic)
    
    def parse_params(self):
        """ parse params into param wrapper class
        """              
        for item in self.client_config.get("parameters"):
            param = Param(item)
            self.client_params_list.append(param)
    
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








            
