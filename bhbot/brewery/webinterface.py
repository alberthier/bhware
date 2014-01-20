# encoding: utf-8


import sys
import os
import math

import bottle
import packets
import position
import statemachine
import commonstates
import binarizer

from definitions import *
from packets import *




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

event_loop = None

app = bottle.Bottle()

@app.route('/<filename>')
def serve_static(filename):
    return bottle.static_file(filename, root = os.path.join(os.path.dirname(__file__), "web"))

@app.route()
def statemachine():
    html = """<!DOCTYPE html>
<html>
<body>
<div>
<h2>State stack:</h2>
<ul>
"""
    state = event_loop.get_current_state()
    while state != event_loop.root_state:
        html += "<li>{}</li>\n".format(type(state).__name__)
        state = state.parent_state
    html += """</ul>
</div>
<div>
<h2>State history:</h2>
<ul>
"""
    previous = None
    for state in event_loop.state_history:
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


@app.route()
def logs():
    html = """<!DOCTYPE html>
<html>
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

@app.route()
def packet_wizard_2():

    packet_type = PACKETS_BY_TYPE[int(vars["packet_type"])]
    code=CodeBuilder().build_sample_code(packet_type)

    html = """<!DOCTYPE html>
<html>
<body>
<div>
<h2>Packet wizard</h2>
<form action="packet_wizard_3">
<textarea rows="4" cols="50" name="code">{sample_code}</textarea>
<button action="submit">go !</button>
</form>

</div>
</body>
</html>
"""
    return html.format(sample_code=code)

@app.route()
def packet_wizard_1():
    html = """<!DOCTYPE html>
<html>
<body>
<div>
<h2>Packet wizard : choose packet type</h2><hr/>
"""
    html += """<form action="packet_wizard_2">"""
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


def packet_wizard_3():
    packet=None

    try :
        code = vars["code"]
        packet = eval(code)
        event_loop.send_packet(packet)
        ret = "OK<br>"
        ret+="<a href=packet_wizard_2?packet_type={}>Send packet of same type</a>".format(packet.TYPE)
    except Exception as e :
        ret = str(e)

    html = "Return : <br>"+ret+"<br>"
    html+="<a href=packet_wizard_1>Send another packet</a>".format(packet.TYPE)
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


def remote_control():
    html = """<!DOCTYPE html>
<html>
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
