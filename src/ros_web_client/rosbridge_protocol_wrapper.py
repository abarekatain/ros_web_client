import json
import rospy


from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.internal import pngcompression

class ros_protocol():

    def __init__(self,id):
        self._protocol = RosbridgeProtocol(0) 
        self._protocol.outgoing = self.rosbridge_outgoing
        self.png_topics = []
        self.msgpack_topics = []

    def initialize_topic(self,wrapper=None,message=None):
        if wrapper is not None:
            #rospy.loginfo("from initialized_topic")
            self.update_compression(wrapper.name,wrapper.compression)            
            self.incoming(wrapper.advertise_command())
        elif message is not None:
            self.update_compression(message["topic"],message.get("_compression",'none'))

    def update_compression(self,id,value):
        if value == "png":
            #rospy.loginfo("update compression png")
            self.png_topics.append(id)
        elif value == "msgpack":
            self.msgpack_topics.append(id)

    def decompress(self,id,data):
        if id in self.png_topics:
            #rospy.loginfo("decompression png")
            data = pngcompression.decode(data)
            return json.loads(data)
        else:
            return data

    def compress(self,id,data):
        if id in self.png_topics:
            #rospy.loginfo("compression png")
            data = json.dumps(data)
            return pngcompression.encode(data)
        else: 
            return data    

    def incoming(self,message):
        #rospy.loginfo("incoming")
        if isinstance(message, str):
            message = json.loads(message)
        #rospy.loginfo(json.dumps(message))
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
            message["msg"] = self.compress(message["topic"],message.get("msg"))
            #rospy.loginfo("outgoing")
        message = json.dumps(message)
        #rospy.loginfo(message)
        self.outgoing(message)

    def outgoing(self,message):
        pass