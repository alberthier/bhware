# encoding: utf-8


import sys
import os
import nanow

import packets
import position
import statemachine
import commonstates
import binarizer

from definitions import *

from packets import *

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

class CodeBuilder:

    def get_sample_value(self, packet_type):
        if hasattr(packet_type, "enum"):
            return list(packet_type.enum.lookup_by_name.keys())[0]
        return None

    def build_sample_code(self, packet_type):
        parameters = self.build_parameters(packet_type)
        return "{class_name}({parameters})".format(class_name=packet_type.__name__, parameters=parameters)

    def build_parameters(self, packet_type):
        ret = []
        for dfn in packet_type.DEFINITION:
            name, type_, default_value = None, None, None
            if len(dfn) == 3 :
                name, type_, default_value = dfn
            else :
                name, type_ = dfn
            ret.append("{name}={val}".format(name=name, val=self.build_value(type_, default_value)))
        return ", ".join(ret)

    def build_value(self, type_, default_value):
        return default_value or self.get_sample_value(type_)


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
      <li><a href="packet_wizard_1" target="linktarget">Packet Wizard</a></li>
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

    def packet_wizard_2(self, headers, vars):

        packet_type = PACKETS_BY_TYPE[int(vars["packet_type"])]
        code=CodeBuilder().build_sample_code(packet_type)

        html = """<!DOCTYPE html>
<html>
<head>
  <title>Packet wizard</title>
  <link rel="stylesheet" type="text/css" href="static/bhweb.css" />
</head>
<body>
  <div>
    <h2>Packet wizard</h2>
    <form action="/packet_wizard_3">
    <textarea rows="4" cols="50" name="code">{sample_code}</textarea>
    <button action="submit">go !</button>
    </form>

  </div>
</body>
</html>
"""
        return html.format(sample_code=code)


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
            for name, item in packet.DEFINITION:
                if not "simulator" in name.lower() :
                    html += self.build_element_from_item(item, name)
            html += """</table>"""
            html += """<input type="hidden" name="packet" value="{}"/><br/>""".format(packet.__name__)
            html += """<input type="submit" value="Send"/><br/>"""
            html += """</form><hr/>"""
        html += """</div>
</body>
</html>
"""
        return html

    def packet_wizard_1(self, headers, vars):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Packets</title>
  <link rel="stylesheet" type="text/css" href="static/bhweb.css" />
</head>
<body>
  <div>
    <h2>Packet wizard : choose packet type</h2><hr/>
"""
        html += """<form action="/packet_wizard_2">"""
        html += """<select name=packet_type>"""
        for packet in sorted(packets.PACKETS_LIST, key=lambda x : x.__name__):
                html += "<option value='{}'>{}</option>".format(packet.TYPE, packet.__name__)
        html += """</select>"""
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
        if isinstance(item, binarizer.Bool):
            if item.default_value:
                checked = 'checked="checked"'
            else:
                checked = ""
            html += """<tr><td>{} ({})</td><td><input type="checkbox" name="{}" {}/></td></tr>""".format(name, item.C_TYPE, name, checked)
        elif isinstance(item, binarizer.Enum8) or isinstance(item, binarizer.Enum8):
            html += """<tr><td>{} ({})</td><td><select name="{}">""".format(name, item.C_TYPE, name)
            for value, name in item.enum.lookup_by_value.items():
                html += """<option value="{}">{} ({})</option>""".format(value, name, value)
            html += """</select></td></tr>"""
        elif isinstance(item, packets.OptionalAngle):
            html += """<tr><td>{}.angle (f)</td><td><input type="text" name="{}.angle" value="{}"/></td></tr>""".format(name, name, item.default_value)
            html += """<tr><td>{}.use_angle (B)</td><td><input type="checkbox" name="{}.use_angle" checked="checked"/></td></tr>""".format(name, name)
        # elif isinstance(item, binarizer.List):
        #     html += """<tr><td>{}.count (B)</td><td><input type="text" name="{}.count" value="{}"/></td></tr>""".format(name, name, 1)
        #     for sub_item_index in range(item.max_count):
        #         html += self.build_element_from_item(item.element_type, "{}[{}]".format(name, sub_item_index))
        else:
            html += """<tr><td>{} ({})</td><td><input type="text" name="{}" value="{}"/></td></tr>""".format(name, item.C_TYPE, name, item.default_value)
        return html


    def packet_wizard_3(self, headers, vars):
        packet=None

        try :
            code = vars["code"]
            packet = eval(code)
            self.eventloop.send_packet(packet)
            ret = "OK<br>"
            ret+="<a href=/packet_wizard_2?packet_type={}>Send packet of same type</a>".format(packet.TYPE)
        except Exception as e :
            ret = str(e)

        html = "Return : <br>"+ret+"<br>"
        html+="<a href=/packet_wizard_1>Send another packet</a>".format(packet.TYPE)
        return html


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
            packet.direction = DIRECTION_FORWARDS
            packet.points = [ pose ]
            self.eventloop.send_packet(packet)
        elif command == "Move":
            distance = float(vars["distance"])
            pose = position.Pose()
            packet = packets.Goto()
            if "forward" in vars:
                packet.direction = DIRECTION_FORWARDS
            else:
                packet.direction = DIRECTION_BACKWARDS
            pose.x = self.eventloop.robot.pose.x + packet.direction * math.cos(self.eventloop.robot.pose.angle) * distance
            pose.y = self.eventloop.robot.pose.y + packet.direction * math.sin(self.eventloop.robot.pose.angle) * distance
            packet.movement = MOVEMENT_MOVE
            packet.points = [ pose ]
            self.eventloop.send_packet(packet)
        return self.remote_control()
