import json

from twisted.internet import defer


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