#!/usr/bin/env python
# encoding: utf-8


import sys
import os

sys.path.append(os.path.dirname(__file__))

import cherrypy




class BHWeb(object):

    def __init__(self, eventloop):
        self.eventloop = eventloop

    @cherrypy.expose
    def index(self):
        return "Hello World!\nCurrent state: {0}".format(type(self.eventloop.get_current_state()).__name__)




def create_app(eventloop):
    return cherrypy.tree.mount(BHWeb(eventloop), script_name = "/")
