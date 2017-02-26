#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright © 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
import sys
PPRZLINK_LIB = os.getenv("PPRZLINK_LIB")
sys.path.append(PPRZLINK_LIB + "/lib/v1.0/python")

from ivy_msg_interface import PprzMessage


class IvyMessageAnswerer(object):
    """ Message Answerer for Paparazzi IVY *_REQ messages"""

    def __init__(self, sender_name, callback, message_class, message_name,
                 ivy_messages_interface, verbose=False):
        """ Arguments :
            senderName : name of the sender of the answers.
            callback : callback function  (*args,**kwargs) -> PaparazziMessage
                        Arguments :
                            *args :
                                sender : is the name of the requester
                                msg : the request message as PaparazziMessage
                                resp : the response massage as PaparazziMessage

                            **kwargs :
                                'id' : the message unique id
                        Must return a PaparazziMessage of same name as resp
                        or None.
        """
        self._message_class = message_class
        self._callback = callback
        self._sender_name = sender_name
        self._ivy_mess_if = ivy_messages_interface

        self._bid = regx = '^(.*?) (.*?) ({})_REQ (.*)'.format(message_name)
        self._ivy_mess_if.bind_raw(self.on_ivy_msg, regx)

        self.verbose = verbose

    def __del__(self):
        self._ivy_mess_if.unbind(self._bid)

    def on_ivy_msg(self, agent, *larg):
        """
            IVY callback.
        """

        assert(len(larg) == 4)

        sender = larg[0]
        mid = larg[1]
        rname = larg[2]
        mname = "{}_REQ".format(rname)
        margs = larg[3].split(' ')

        msg = PprzMessage(self._message_class, mname)
        msg.set_values(margs)
        kwargs = {'id': mid}

        resp = PprzMessage(self._message_class, larg[2])
        resp = self._callback(sender=sender, msg=msg, resp=resp, **kwargs)

        if resp is None:
            if self.verbose:
                print("WARNING IvyMessageAnswerer.on_ivy_message :"
                      "No response sent")
            return

        if not(isinstance(resp, PprzMessage)) or resp.name != rname:
            raise Exception("Invalid message returned by request callback")

        respstr = "{} {} {} {}".format(mid, self._sender_name, resp.name,
                                       resp.payload_to_ivy_string())

        self._ivy_mess_if.send(respstr)
