#!/usr/bin/env python
# encoding: utf-8

import sys
import os
import random

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "simulator"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "common"))

from PyQt4.QtGui import *

from mainwindow import *




if __name__ == "__main__":

    random.seed()

    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("simulator/icons/main.png"))

    mw = MainWindow()
    mw.show()

    sys.exit(app.exec_())

