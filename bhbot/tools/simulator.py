#!/usr/bin/env python3
# encoding: utf-8
from argparse import ArgumentParser

import sys
import os
import random

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "simulator"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "common"))

from PyQt4.QtGui import *

from mainwindow import *




if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("--pydev-debug", nargs=2)
    parser.add_argument("--main-fsm", default = None)
    parser.add_argument("--secondary-fsm", default = None)

    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("simulator/icons/main.png"))

    mw = MainWindow(args)
    mw.show()

    sys.exit(app.exec_())

