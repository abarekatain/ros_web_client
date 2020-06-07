import json
import rospy


from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.internal import pngcompression

import umsgpack

class ros_protocol():

    def __init__(self,id):
        self._protocol = RosbridgeProtocol(id) 
        self._protocol.outgoing = self.rosbridge_outgoing
        self.topics_compression_state = {}
        self.topics_serialization_state = {}

    def initialize_topic(self,wrapper=None,message=None):
        if wrapper is not None:
            topic = wrapper.name
            compression = wrapper.compression
            serialization = wrapper.serialization          
        elif message is not None:
            topic = message["topic"]
            compression = message.get("_compression",'none')  
            serialization = message.get("_serialization","json")          

        self.update_compression(topic,compression)
        self.update_serialization(topic,serialization)
        
    def update_compression(self,topic,value):
        self.topics_compression_state[topic] = value
    
    def update_serialization(self,topic,value):
        self.topics_serialization_state[topic] = value

    def decompress(self,topic,data):
        if self.topics_compression_state[topic] == "png":
            data = pngcompression.decode(data)
            return json.loads(data)
        else:
            return data

    def compress(self,topic,data):
        if self.topics_compression_state[topic] == "png":
            data = json.dumps(data)
            return pngcompression.encode(data)
        else: 
            return data 

    def serialize(self,topic,data):
        if self.topics_serialization_state[topic] == "msgpack":
            data = umsgpack.packb(data)
            return data
        else:
            return json.dumps(data) 
    
    def deserialize(self,data):
        data = umsgpack.unpackb(data)
        return data


    def incoming(self,message):
        if isinstance(message, str):
            message = json.loads(message)
        elif isinstance(message, bytes):
            message = self.deserialize(message)


        if message.get("op") == "subscribe":
            self.initialize_topic(message=message)
        elif message.get("op") == "publish":
            message["msg"] = self.decompress(message["topic"],message.get("msg"))


        message = json.dumps(message)
        #--------
        #replace JSON Null values in float32 types with infinity datatype (changed according to the error for LaserScan values)
        message = message.replace("null", "Infinity")
        #--------
        self._protocol.incoming(message)

    def rosbridge_outgoing(self,message):

        message = json.loads(message)
        if message.get("op") == "publish":
            topic_name = message["topic"]
            message["msg"] = self.compress(topic_name,message.get("msg"))
            message = self.serialize(topic_name,message)
        
        if isinstance(message, bytes):
            self.outgoing(message,isBinary=True,identifier=topic_name)
        elif isinstance(message,str):
            self.outgoing(message)
        else:
            message = json.dumps(message)
            self.outgoing(message)

    def outgoing(self,message,isBinary=False,identifier=None):
        pass