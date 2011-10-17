#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic
import os
import imp

import logger
import packets

import logview
import logfieldview



class MainWindowController(QObject):

    def __init__(self, log_file):
        QObject.__init__(self)

        self.ui = uic.loadUi(os.path.splitext(__file__)[0] + ".ui")
        self.ui.setWindowIcon(QIcon.fromTheme("text-x-generic"))

        self.logview_controller = logview.LogViewController(self.ui)
        self.fieldview_controller = logfieldview.LogFieldViewController(self.ui)

        self.load_log(log_file)

        self.ui.show()


    def load_log(self, log_file):
        log = imp.load_source("logcontent", log_file).log
        lineno = 1
        last_lineno = len(log)
        for log_line in log:
            packet_type = log_line[logger.LOG_DATA_TYPE]
            if packets.PACKETS_BY_NAME.has_key(packet_type):
                packet = packets.PACKETS_BY_NAME[packet_type]()
                packet.from_dict(log_line[logger.LOG_DATA_PACKET])
                log_line[logger.LOG_DATA_PACKET] = packet
            self.logview_controller.process_log_line(log_line, lineno, last_lineno)
            self.fieldview_controller.process_log_line(log_line, lineno, last_lineno)
            lineno += 1
