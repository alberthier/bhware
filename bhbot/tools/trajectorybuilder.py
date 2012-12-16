#!/usr/bin/env python3
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import os
import sys

# Update PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "common"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "trajectorybuilder"))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery"))

from mainwindow import *




if __name__ == "__main__":
    app = QApplication(sys.argv)

    mw = MainWindowController()

    sys.exit(app.exec_())
