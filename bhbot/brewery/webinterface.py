# encoding: utf-8


import json
import os
import socket
import traceback
import urllib

import bottle
import statemachine


event_loop = None
app = bottle.Bottle()


@app.route('/hostname')
def hostname():
    return socket.gethostname()


@app.route('/statemachines')
def hostname():
    result = {}
    for fsm in event_loop.fsms:
        states = []
        result[fsm.name] = states
        for state in fsm.state_stack:
            states.append(state.name)
    return json.dumps(result)


@app.route('/remotecontrol')
def remotecontrol():
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


@app.post("/eval")
def eval():
    encoding = "iso-8859-1"
    if "Content-Type" in bottle.request.headers:
        for key, value in urllib.parse.parse_qsl(bottle.request.headers["Content-Type"]):
            if key == "charset":
                encoding = value
                break
    fsm = statemachine.StateMachine(event_loop, "eval", code = str(bottle.request.body.read(), encoding))
    text = ""
    if fsm.error is not None:
        bottle.response.status = 500
        for l in traceback.format_exception(type(fsm.error), fsm.error, None):
            text += l
    else:
        text = "OK"
    return text
