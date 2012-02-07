#!/usr/bin/env python
# encoding: utf-8


import sys
import os
import socket

sys.path.append(os.path.dirname(__file__))

import cherrypy

import packets
import trajectory
from definitions import *




class BHWeb(object):

    def __init__(self, eventloop):
        self.eventloop = eventloop

    @cherrypy.expose
    def index(self):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>BH Team robot web interface</title>
  <link rel="stylesheet" type="text/css" href="bhweb.css" />
</head>
<body>
  <div class="header">
    <h1>BH Team</h1>
    <ul>
      <li><a href="statemachine" target="linktarget">State Machine</a></li>
      <li><a href="logs" target="linktarget">Logs</a></li>
      <li><a href="packets" target="linktarget">Packets</a></li>
      <li><a href="http://{}:42080" target="linktarget">PIC</a></li>
    </ul>
  </div>
  <iframe src="statemachine" name="linktarget" />
</body>
</html>
""".format(socket.gethostname())
        return html


    @cherrypy.expose
    def statemachine(self):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Statemachine</title>
  <link rel="stylesheet" type="text/css" href="bhweb.css" />
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
</body>
</html>
"""
        return html


    @cherrypy.expose
    def logs(self):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Logs</title>
  <link rel="stylesheet" type="text/css" href="bhweb.css" />
</head>
<body>
  <div>
    <h2>Logs:</h2>
    <ul>
"""
        files = os.listdir(LOG_DIR)
        files.sort()
        for f in reversed(files):
            html += '<li><a href="logs/{0}">{0}</a></li>\n'.format(f)

        html += """</ul>
  </div>
</body>
</html>
"""
        return html


    @cherrypy.expose
    def packets(self):
        html = """<!DOCTYPE html>
<html>
<head>
  <title>Packets</title>
  <link rel="stylesheet" type="text/css" href="bhweb.css" />
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
        if name == None:
            name = item.name
        if isinstance(item, packets.BoolItem):
            if item.default_value:
                checked = 'checked="checked"'
            else:
                checked = ""
            html += """<tr><td>{} ({})</td><td><input type="checkbox" name="{}" {}/></td></tr>""".format(name, item.C_TYPE, name, checked)
        elif isinstance(item, packets.Enum8Item) or isinstance(item, packets.UEnum8Item):
            html += """<tr><td>{} ({})</td><td><select name="{}">""".format(name, item.C_TYPE, name)
            for value, name in item.enum.lookup_by_value.iteritems():
                html += """<option value="{}">{} ({})</option>""".format(value, name, value)
            html += """</select></td></tr>"""
        elif isinstance(item, packets.PoseWithOptionalAngleItem):
            html += """<tr><td>{}.x (f)</td><td><input type="text" name="{}.x" value="{}"/></td></tr>""".format(name, name, item.default_value.x)
            html += """<tr><td>{}.y (f)</td><td><input type="text" name="{}.y" value="{}"/></td></tr>""".format(name, name, item.default_value.y)
            if item.default_value.angle != None:
                default_angle = item.default_value.angle
            else:
                default_angle = "0.0"
            html += """<tr><td>{}.angle (f)</td><td><input type="text" name="{}.angle" value="{}"/></td></tr>""".format(name, name, default_angle)
            if not isinstance(item, packets.PoseItem):
                html += """<tr><td>{}.use_angle (B)</td><td><input type="checkbox" name="{}.use_angle" checked="checked"/></td></tr>""".format(name, name)
        elif isinstance(item, packets.ListItem):
            html += """<tr><td>{}.count (B)</td><td><input type="text" name="{}.count" value="{}"/></td></tr>""".format(name, name, 1)
            for sub_item_index in xrange(item.max_elements):
                html += self.build_element_from_item(item.element_type, "{}[{}]".format(name, sub_item_index))
        else:
            html += """<tr><td>{} ({})</td><td><input type="text" name="{}" value="{}"/></td></tr>""".format(name, item.C_TYPE, name, item.default_value)
        return html


    @cherrypy.expose
    def send_packet(self, **kwargs):
        packet_type = packets.PACKETS_BY_NAME[kwargs["packet"]]
        packet = packet_type()
        for item in packet_type.DEFINITION:
            setattr(packet, item.name, self.build_item_from_query(kwargs, item))
        self.eventloop.send_packet(packet)
        return self.packets()


    def build_item_from_query(self, query, item, name = None):
        if name == None:
            name = item.name
        if isinstance(item, packets.BoolItem):
            return query.has_key(name)
        elif isinstance(item, packets.FloatItem):
            return float(query[name])
        elif isinstance(item, packets.PoseWithOptionalAngleItem):
            pose = trajectory.Pose()
            pose.x = float(query[name + ".x"])
            pose.y = float(query[name + ".y"])
            if isinstance(item, packets.PoseItem) or query.has_key(name + ".use_angle"):
                pose.angle = float(query[name + ".angle"])
            else:
                pose.angle = None
            return pose
        elif isinstance(item, packets.ListItem):
            count = int(query[name + ".count"])
            l = []
            for sub_item_index in xrange(count):
                elt = self.build_item_from_query(query, item.element_type, "{}[{}]".format(name, sub_item_index))
                l.append(elt)
            return l
        else:
            return int(query[name])




def create_app(eventloop):
    return cherrypy.tree.mount(BHWeb(eventloop), script_name = "/", config = {
        "/" : {
            "tools.staticdir.on"  : True,
            "tools.staticdir.dir" : os.path.join(os.path.dirname(__file__), "static"),
        },
        "/logs" : {
            "tools.staticdir.on"  : True,
            "tools.staticdir.dir" : LOG_DIR,
            "tools.staticdir.content_types" : { "py": "text/plain" },
        }
    })
