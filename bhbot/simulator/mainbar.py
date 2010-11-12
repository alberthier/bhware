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
        self.start.setIcon(QIcon.fromTheme("media-playback-start"))
        self.stop.setIcon(QIcon.fromTheme("media-playback-stop"))
        self.start.clicked.connect(self.startGame)
        self.stop.clicked.connect(self.stopGame)


    def startGame(self):
        print "Start game"


    def stopGame(self):
        print "Stop game"
