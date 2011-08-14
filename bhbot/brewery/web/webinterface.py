#!/usr/bin/env python
# encoding: utf-8


import sys
import os

sys.path.append(os.path.dirname(__file__))

import cherrypy




class BHWeb(object):

    @cherrypy.expose
    def index(self):
        return "Hello World!"




app = cherrypy.tree.mount(BHWeb(), script_name = "/")
