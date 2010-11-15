#!/usr/bin/env python
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic
import os
import sys
import imp

# Update PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "common"))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery"))

import helpers

from logview import *

if __name__ == "__main__":
    app = QApplication(sys.argv)

    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        log_file = helpers.get_last_logfile()

    mw = MainWindowController(log_file)

    sys.exit(app.exec_())
