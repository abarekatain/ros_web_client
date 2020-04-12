import json

from ros_web_client.message_wrapper import Topic
class ConfigParser(object):
    """ Parser for Initialization config file in JSON

    Args:
        config (:obj: 'str') content of config file in JSON format
    """

    def __init__(self, config):
        self.config = json.loads(config)
        self.list_topics = []
    
    def parse_topics(self):
        for item in self.config.get("topics"):
            topic=Topic(item["name"],item["message_type"],parameters=item)
            self.list_topics.append(topic)

            
