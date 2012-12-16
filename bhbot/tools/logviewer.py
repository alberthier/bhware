#!/usr/bin/env python3
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import os
import sys

# Update PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "common"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "logviewer"))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery"))

import helpers

from mainwindow import *

if __name__ == "__main__":
    app = QApplication(sys.argv)

    port = None
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
        if log_file == "--remote":
            if len(sys.argv) > 2:
                remote_addr = sys.argv[2]
                idx = remote_addr.find(':')
                if idx != -1:
                    log_file = remote_addr[:idx]
                    port = int(remote_addr[idx + 1:])
                else:
                    log_file = remote_addr
                    port = 80
            else:
                print("Remove address missing. ex: 192.168.0.42:8000")
    else:
        log_file = helpers.get_last_logfile()

    mw = MainWindowController(log_file, port)

    sys.exit(app.exec_())
