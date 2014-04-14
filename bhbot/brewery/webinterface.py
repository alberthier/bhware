# encoding: utf-8


import json
import os
import socket
import traceback
import urllib.parse

import statemachine


class WebInterface:

    def __init__(self, event_loop):
        self.event_loop = event_loop

    def __call__(self, environ, start_response):
        response = None
        path = environ["PATH_INFO"]
        if path == "/hostname":
            response = socket.gethostname()
        elif path == "/statemachines":
            response = self.statemachines(environ)
        elif path == "/remotecontrol":
            response = self.remotecontrol(environ)
        elif path == "/eval":
            response = self.eval(environ)

        if response is None:
            code = "404 Not Found"
            response = ""
        else:
            code = "200 OK"

        start_response(code, [('Content-type','text/plain; charset=utf-8')])
        return [bytes(response, "utf-8")]


    def hostname(self, environ):
        return socket.gethostname()


    def statemachines(self, environ):
        result = {}
        for fsm in self.event_loop.fsms:
            states = []
            result[fsm.name] = states
            for state in fsm.state_stack:
                states.append(state.name)
        return json.dumps(result)


    def remotecontrol(self, environ):
        result = {}
        packets = []
        packets.append("self.send_packet(EnableAntiBlocking(1))")
        packets.append("self.send_packet(EnableAntiBlocking(2))")
        packets.append("self.send_packet(EnableAntiBlocking(3))")
        packets.append("self.send_packet(EnableAntiBlocking(4))")
        packets.append("self.send_packet(EnableAntiBlocking(5))")
        packets.append("self.send_packet(EnableAntiBlocking(6))")
        result["Packets"] = packets
        states = []
        states.append("self.send_packet(EnableAntiBlocking(7))")
        states.append("self.send_packet(EnableAntiBlocking(8))")
        states.append("self.send_packet(EnableAntiBlocking(9))")
        states.append("self.send_packet(EnableAntiBlocking(10))")
        states.append("self.send_packet(EnableAntiBlocking(11))")
        states.append("self.send_packet(EnableAntiBlocking(12))")
        result["States"] = states
        return json.dumps(result)


    def eval(self, environ):
        encoding = "iso-8859-1"
        if "CONTENT_TYPE" in environ:
            for key, value in urllib.parse.parse_qsl(environ["CONTENT_TYPE"]):
                if key == "charset":
                    encoding = value
                    break
        code = str(environ["wsgi.input"].read(), encoding)
        fsm = statemachine.StateMachine(self.event_loop, "eval", code = code)
        text = ""
        if fsm.error is not None:
            for l in traceback.format_exception(type(fsm.error), fsm.error, None):
                text += l
        else:
            text = "OK"
        return text
