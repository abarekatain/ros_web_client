# MIT License
#
# Copyright (c) 2020 Alireza Barekatain - Isfahan University of Technology
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import json
import rosparam

from ros_web_client.message_wrapper import Topic, Service, Param


class CommandHandler(object):
    """ Handle Network RPC Callbacks
        (Internal)

        This class stores all the callbacks to be called after sending initialization commands
        to the client
    """

    def callback_topic_client(self,command,result):
        """ Callback When a Topic is Subscribed on Client and Needs to Be Advertised on Server

            Args:
            command: sent command of Type Command -- see Command Class
            result: returned argument from client
        """
        if result is None:
            #isIncoming variable declares whether the topic messages are to be received from external source
            command.protocol.initialize_topic(wrapper=command.wrapper,isIncoming=True)
            command.protocol.incoming(command.wrapper.advertise_command())

    def callback_topic_server(self,command,result):
        """ Callback When a Topic is Advertised on Client and Needs to Be Subscribed on Server

            Args:
            command: sent command of Type Command -- see Command Class
            result: returned argument from client
        """
        if result is None:
            #isIncoming variable declares whether the topic messages are to be received from external source
            command.protocol.initialize_topic(wrapper=command.wrapper,isIncoming=False)
            command.protocol.incoming(command.wrapper.subscribe_command())

    def callback_param_client(self,command,result):
        """ Callback When a Parameter is Fetched from Client and Needs to Be Set on Server

            Args:
            command: sent command of Type Command -- see Command Class
            result: returned argument from client
        """
        if result is not None:
            result=json.loads(result)
            param_value = result["values"]["value"]
            rosparam.set_param(command.wrapper.name,param_value)
    
    def callback_service_client(self,command,result):
        """ Callback When a Service is Available on Client and Needs to be Advertised on Server

            Args:
            command: sent command of Type Command -- see Command Class
            result: returned argument from client
        """
        if result is None:
            command.protocol.initialize_service(wrapper=command.wrapper)
            command.protocol.incoming(command.wrapper.advertise_command())
    
    def callback_service_server(self,command,result):
        """ Callback When a Service is Advertised on Client and Needs to be Available on Server

            Args:
            command: sent command of Type Command -- see Command Class
            result: returned argument from client
        """
        pass
        #if result is None:
            #command.protocol.incoming(command.wrapper.request_command())

class ConfigHandler(object):
    """ Generate Appropriate Initialization Commands from The Initialization Config,
        to send to the client
        (External Use)

        Attributes:
        config: Content of initialization config file
        protocol: rosbridge-level protocol wrapper instance

        Methods:
        return_commands
    """

    def __init__(self,config,protocol):
        self.protocol = protocol
        self.parser = ConfigParser(config)
        self.command_handler = CommandHandler()
        self.commands_list = []

    def _handle_topics(self):
        """ Wrap each topic in config list in a Command Class and append to a command list 
            for the server session to call
        """
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

    def _handle_services(self):
        """ Wrap each service in config list in a Command Class and append to a command list
            for the server session to call
        """
        #Client
        for service in self.parser.client_services_list:
            command = Command(service.request_command(),
                        self.command_handler.callback_service_client,
                        wrapper=service,protocol=self.protocol)
            self.commands_list.append(command)

        #Server
        for service in self.parser.server_services_list:
            command = Command(service.advertise_command(),
                        self.command_handler.callback_service_server,
                        wrapper=service,protocol=self.protocol)
            self.commands_list.append(command)
    
    def _handle_params(self):
        """ Wrap each parameter fetch request in config list in a Command Class
            and append to a command list for the server session to call
        """
        #Client
        for param in self.parser.client_params_list:
            command = Command(param.get_command(),
                        self.command_handler.callback_param_client,
                        wrapper=param,protocol=self.protocol)            
            self.commands_list.append(command)
    
    def return_commands(self):
        """ Generate all the commands required for initialization

            Returns:
            A list of all the commands (of type Command) to be passed to client
        """
        self._handle_params()
        self._handle_services()
        self._handle_topics()
        return self.commands_list

class ConfigParser(object):
    """ Parser for Initialization config file in JSON format
        (Internal)

        Args:
        config: content of config file in JSON format

        Methods:
        parse()

        Variables:
        client_topics_list
        client_services_list
        client_params_list
        server_topics_list
        server_services_list
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

    def parse_services(self):
        """ parse services into Service wrapper class
        """
        #Client
        for item in self.client_config.get("services"):
            service = Service(item["name"],type=item["type"],parameters=item)
            self.client_services_list.append(service)        

        #Server
        for item in self.server_config.get("services"):
            service = Service(item["name"],type=item["type"],parameters=item)
            self.server_services_list.append(service) 

    def parse_params(self):
        """ parse params into param wrapper class
        """              
        for item in self.client_config.get("parameters"):
            param = Param(item)
            self.client_params_list.append(param)
    
    def parse(self):
        """ Parse Configurations into their own message_wrapper classes
        """
        self.parse_topics()
        self.parse_params()
        self.parse_services()


class Command(object):
    """ Command object to wrap all necessary data for RPC Call
        (Internal)
        
        Args:
        message: the command message to send
        callback: callback function to handle response
        wrapper: message wrapper object, Topic, Service, or Param
        protocol: rosbridge-level protocol wrapper instance
    """

    def __init__(self,message,callback,wrapper=None,protocol=None):
        self.message = message
        self.callback = callback
        self.wrapper = wrapper
        self.protocol = protocol








            
