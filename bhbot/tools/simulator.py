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

    args = parser.parse_args()

    debug_host = None
    debug_port = 0

    if args.pydev_debug :
        debug_host, debug_port = args.pydev_debug
        debug_port = int(debug_port)

    random.seed()

    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("simulator/icons/main.png"))

    if len(sys.argv) > 1:
        piece_config = sys.argv[1]
    else:
        piece_config = ""

    mw = MainWindow(piece_config, debug_host = debug_host, debug_port =  debug_port)
    mw.show()

    sys.exit(app.exec_())

