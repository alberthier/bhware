# encoding: utf-8


import sys
import os
import nanow

import packets
import position
import statemachine
import commonstates
from definitions import *

import math


class WebState(statemachine.State):

    def __init__(self, event_loop):
        statemachine.State.__init__(self)
        self.event_loop = event_loop
        self.old_root_state = self.event_loop.root_state
        self.event_loop.root_state = self


    #noinspection PyUnusedLocal
    def on_exit_substate(self, substate):
        self.event_loop.root_state = self.old_root_state




class BHWeb(object):


    static = nanow.StaticDir(os.path.join(os.path.dirname(__file__), "static"))


    def __init__(self, eventloop):
        """
        @type eventloop: EventLoop
        """
        self.eventloop = eventloop


    def execute_statemachine(self, state):
        ws = WebState(self.eventloop)
        ws.switch_to_substate(state)


    def index(self, headers, vars):
        host = headers["Host"].split(":")[0]
        html = """<!DOCTYPE html>
<html>
<head>
  <title>BH Team robot web interface</title>
  <link rel="stylesheet" type="text/css" href="static/bhweb.css" />
</head>
<body>
  <div class="header">
    <h1>BH Team</h1>
    <ul>
      <li><a href="statemachine" target="linktarget">State Machine</a></li>
      <li><a href="logs" target="linktarget">Logs</a></li>
      <li><a href="packets" target="linktarget">Packets</a></li>
      <li><a href="remote_control" target="linktarget">Remote Control</a></li>
      <li><a href="http://{}:42080" target="linktarget">PIC</a></li>
    </ul>
  </div>
  <iframe src="statemachine" name="linktarget" />
</body>
</html>
""".format(host)
        return html


    def statemachine(self, headers, vars):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Statemachine</title>
  <link rel="stylesheet" type="text/css" href="static/bhweb.css" />
</head>
<body>
  <div>
    <h2>State stack:</h2>
    <ul>
"""
        state = self.eventloop.get_current_state()
        while state != self.eventloop.root_state:
            html += "<li>{}</li>\n".format(type(state).__name__)
            state = state.parent_state
        html += """</ul>
  </div>
  <div>
    <h2>State history:</h2>
    <ul>
"""
        previous = None
        for state in self.eventloop.state_history:
            if state.parent_state == previous :
                html+="<ul>"
            elif previous and previous.parent_state == state :
                html+="</ul>"
            html += "<li>{}</li>\n".format(type(state).__name__)
            previous = state
        html += """</ul>
  </div>
</body>
</html>
"""
        return html


    def logs(self, headers, vars):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Logs</title>
  <link rel="stylesheet" type="text/css" href="static/bhweb.css" />
</head>
<body>
  <div>
    <h2>Logs:</h2>
    <ul>
"""
        files = os.listdir(LOG_DIR)
        files.sort()
        for f in reversed(files):
            if f.endswith(".py"):
                html += '<li><a href="logs/{0}">{0}</a></li>\n'.format(f)

        html += """</ul>
  </div>
</body>
</html>
"""
        return html


    def logurls(self, headers, vars):
        text = ""
        files = os.listdir(LOG_DIR)
        files.sort()
        for f in reversed(files):
            if f.endswith(".py"):
                text += 'logs/{0}\n'.format(f)
        return text


    def packets(self, headers, vars):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Packets</title>
  <link rel="stylesheet" type="text/css" href="static/bhweb.css" />
</head>
<body>
  <div>
    <h2>Packets:</h2><hr/>
"""
        for packet in packets.PACKETS_LIST:
            html += """<h3>{} ({})</h3>""".format(packet.__name__, packet.TYPE)
            html += """<form method="POST" action="send_packet">"""
            html += """<table>"""
            for item in packet.DEFINITION:
                html += self.build_element_from_item(item)
            html += """</table>"""
            html += """<input type="hidden" name="packet" value="{}"/><br/>""".format(packet.__name__)
            html += """<input type="submit" value="Send"/><br/>"""
            html += """</form><hr/>"""
        html += """</div>
</body>
</html>
"""
        return html


    def build_element_from_item(self, item, name = None):
        html = ""
        if name is None:
            name = item.name
        if isinstance(item, packets.BoolItem):
            if item.default_value:
                checked = 'checked="checked"'
            else:
                checked = ""
            html += """<tr><td>{} ({})</td><td><input type="checkbox" name="{}" {}/></td></tr>""".format(name, item.C_TYPE, name, checked)
        elif isinstance(item, packets.Enum8Item) or isinstance(item, packets.UEnum8Item):
            html += """<tr><td>{} ({})</td><td><select name="{}">""".format(name, item.C_TYPE, name)
            for value, name in item.enum.lookup_by_value.items():
                html += """<option value="{}">{} ({})</option>""".format(value, name, value)
            html += """</select></td></tr>"""
        elif isinstance(item, packets.OptionalAngle):
            html += """<tr><td>{}.angle (f)</td><td><input type="text" name="{}.angle" value="{}"/></td></tr>""".format(name, name, item.default_value)
            html += """<tr><td>{}.use_angle (B)</td><td><input type="checkbox" name="{}.use_angle" checked="checked"/></td></tr>""".format(name, name)
        elif isinstance(item, packets.ListItem):
            html += """<tr><td>{}.count (B)</td><td><input type="text" name="{}.count" value="{}"/></td></tr>""".format(name, name, 1)
            for sub_item_index in range(item.max_elements):
                html += self.build_element_from_item(item.element_type, "{}[{}]".format(name, sub_item_index))
        else:
            html += """<tr><td>{} ({})</td><td><input type="text" name="{}" value="{}"/></td></tr>""".format(name, item.C_TYPE, name, item.default_value)
        return html


    def send_packet(self, headers, vars):
        packet_type = packets.PACKETS_BY_NAME[vars["packet"]]
        packet = packet_type()
        for item in packet_type.DEFINITION:
            setattr(packet, item.name, self.build_item_from_query(vars, item))
        self.eventloop.send_packet(packet)
        return self.packets()


    def build_item_from_query(self, query, item, name = None):
        if name is None:
            name = item.name
        if isinstance(item, packets.BoolItem):
            return name in query
        elif isinstance(item, packets.FloatItem):
            return float(query[name])
        elif isinstance(item, packets.OptionalAngle):
            if name + ".use_angle" in query:
                angle = float(query[name + ".angle"])
            else:
                angle = None
            return angle
        elif isinstance(item, packets.ListItem):
            count = int(query[name + ".count"])
            l = []
            for sub_item_index in range(count):
                elt = self.build_item_from_query(query, item.element_type, "{}[{}]".format(name, sub_item_index))
                l.append(elt)
            return l
        else:
            return int(query[name])


    def remote_control(self, headers, vars):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Remote Control</title>
  <link rel="stylesheet" type="text/css" href="static/bhweb.css" />
</head>
<body>
    <form method="POST" action="process_remote_control">
        <input type="hidden" name="type" value="GotoStart"/>
        <input type="submit" value="Goto initial position" />
    </form>
    <hr/>
    <form method="POST" action="process_remote_control">
        <input type="hidden" name="type" value="Rotate"/>
        Angle (deg) <input type="text" name="angle" value="45" /><br/>
        <input type="submit" value="Rotate" />
    </form>
    <hr/>
    <form method="POST" action="process_remote_control">
        <input type="hidden" name="type" value="Move"/>
        Distance (m) <input type="text" name="distance" value="0.5" /><br/>
        Forward <input type="checkbox" name="forward" checked="checked" /><br/>
        <input type="submit" value="Move" />
    </form>
    <hr/>
</body>
</html>
"""
        return html


    def process_remote_control(self, headers, vars):
        command = vars["type"]
        if command == "GotoStart":
            self.execute_statemachine(commonstates.GotoHome())
        elif command == "Rotate":
            angle = float(vars["angle"]) / 180.0 * math.pi
            pose = position.Pose()
            pose.x = self.eventloop.robot.pose.x
            pose.y = self.eventloop.robot.pose.y
            pose.angle = self.eventloop.robot.pose.angle + angle
            packet = packets.Goto()
            packet.movement = MOVEMENT_ROTATE
            packet.direction = DIRECTION_FORWARD
            packet.points = [ pose ]
            self.eventloop.send_packet(packet)
        elif command == "Move":
            distance = float(vars["distance"])
            pose = position.Pose()
            packet = packets.Goto()
            if "forward" in vars:
                packet.direction = DIRECTION_FORWARD
            else:
                packet.direction = DIRECTION_BACKWARD
            pose.x = self.eventloop.robot.pose.x + packet.direction * math.cos(self.eventloop.robot.pose.angle) * distance
            pose.y = self.eventloop.robot.pose.y + packet.direction * math.sin(self.eventloop.robot.pose.angle) * distance
            packet.movement = MOVEMENT_MOVE
            packet.points = [ pose ]
            self.eventloop.send_packet(packet)
        return self.remote_control()
