# Copyright (c) 2020, Alireza Barekatain - Isfahan University of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import json

from twisted.internet import defer


class ServiceHandler(object):
    """ Handle Service Requests from RPC
        catch the request and return a deffered,
        the deffered will callback when request is completed

        Attributes:
        protocol : instance of Rosbridge Protocol wrapper

        Methods:
        execute
        callback
    """

    def __init__(self,protocol):
        self.protocol = protocol
        self.service_deffereds={}
   
    
    def execute(self, command):
        """ Process a rosbridge call_service rosbridge command

            Args:
            command : rosbridge call_service message

            Returns:
            a Differed which fires when the service_response is ready
        """
        dict_command=json.loads(command)
        d=defer.Deferred()
        d.addCallback(self._return_result)
        self.service_deffereds[dict_command["id"]]=d
        self.protocol.incoming(command)
        return d

    def callback(self,message):
        """ Call this method when a rosbridge service_response is ready.
            (Fire the Differed associated with the service call and pass the result)

            Args:
            message : rosbridge service_response message

        """
        dict_message=json.loads(message)
        d=self.service_deffereds.get(dict_message["id"])
        d.callback(message)
        del self.service_deffereds[dict_message["id"]]

    def _return_result(self,message):
        return message