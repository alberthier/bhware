#!/usr/bin/env python
# encoding: utf-8


import sys
import os
import socket

sys.path.append(os.path.dirname(__file__))

import cherrypy




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
      <li><a href="http://{0}:42080" target="linktarget">PIC</a></li>
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
  <title>BH Team robot web interface</title>
  <link rel="stylesheet" type="text/css" href="bhweb.css" />
</head>
<body>
  <div>
    <h2>State stack:</h2>
    <ul>
"""
        state = self.eventloop.get_current_state()
        while state != self.eventloop.root_state:
            html += """<li>{0}</li>""".format(type(state).__name__)
            state = state.parent_state
        html + """</ul>
  </div>
  <iframe name="linktarget" />
</body>
</html>
"""
        return html





def create_app(eventloop):
    return cherrypy.tree.mount(BHWeb(eventloop), script_name = "/", config = { "/" : {
        "tools.staticdir.on"  : True,
        "tools.staticdir.dir" : os.path.join(os.path.dirname(__file__), "static"),
    } })
