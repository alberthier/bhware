#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtGui import *
from PyQt4 import uic

(MainBar_Ui, MainBar_Widget) =  uic.loadUiType(os.path.join(os.path.dirname(__file__), "mainbar.ui"))




class MainBar(QWidget, MainBar_Ui):

    def __init__(self, parent = None):
        QWidget.__init__(self, parent)
        MainBar_Ui.__init__(self)
        self.setupUi(self)
        icons_dir = os.path.join(os.path.dirname(__file__), "icons")
        self.set_icon(self.reload, "refresh")
        self.set_icon(self.start_pause, "start")
        self.set_icon(self.stop, "stop")


    def set_icon(self, button, icon_name):
        icons_dir = os.path.join(os.path.dirname(__file__), "icons")
        button.setIcon(QIcon(os.path.join(icons_dir, "{0}.svg".format(icon_name))))
