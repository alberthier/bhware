# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtNetwork import *
from PyQt4 import uic
import os
import imp

import logger
import packets

import logtools
import logview
import logfieldview
import speedview

from definitions import *




class MainWindowController(QObject):

    def __init__(self, log_file, port):
        QObject.__init__(self)

        self.ui = uic.loadUi(os.path.splitext(__file__)[0] + ".ui")
        self.ui.setWindowIcon(QIcon.fromTheme("text-x-generic"))

        self.views = []
        self.views.append(logview.LogViewController(self.ui))
        self.views.append(logfieldview.LogFieldViewController(self.ui))
        self.views.append(speedview.SpeedViewController(self.ui))

        self.log_file = log_file
        self.host = log_file
        self.port = port
        self.network_manager = None
        self.download_dialog = None

        QTimer.singleShot(0, self.fetch_log)

        self.ui.show()


    def fetch_log(self):
        if self.port != None:
            self.download_remote_loglist()
        else:
            self.load_log()


    def load_log(self):
        log = imp.load_source("logcontent", self.log_file).log
        lineno = 1
        last_lineno = len(log)
        for log_line in log:
            content = log_line[logger.LOG_LINE_CONTENT]
            packet_type = log_line[logger.LOG_LINE_PACKET]
            if type(packet_type) != str:
                log_line[logger.LOG_LINE_CONTENT] = eval(content)
            log_line.append(content)
            for view in self.views:
                if packet_type is packets.KeepAlive:
                    match_time = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "match_time")
                    if match_time <= 0 or match_time > 90000:
                        continue
                view.process_log_line(log_line, lineno, last_lineno)
            lineno += 1
        for view in self.views:
            view.log_loaded()



    def download_remote_loglist(self):
        self.network_manager = QNetworkAccessManager()
        self.network_manager.finished.connect(self.display_log_chooser)
        self.network_manager.get(QNetworkRequest(QUrl("http://{}:{}/logurls".format(self.host, self.port))))


    def display_log_chooser(self, reply):
        logs = []
        while reply.canReadLine():
            logs.append(str(reply.readLine()).strip())

        if len(logs) == 0:
            QMessageBox.critical(self.ui, "Error", "No remote log file")
            QApplication.instance().quit();
            return

        self.download_dialog = uic.loadUi(os.path.dirname(__file__) + "/downloaddialog.ui")
        list_view = self.download_dialog.list_view
        list_view.setModel(QStringListModel(logs))
        list_view.selectionModel().select(list_view.model().createIndex(0, 0), QItemSelectionModel.Select)
        if self.download_dialog.exec_() == QDialog.Accepted:
            selected = list_view.selectionModel().currentIndex().row()
            if selected != -1:
                self.network_manager.finished.disconnect(self.display_log_chooser)
                self.network_manager.finished.connect(self.save_remote_log)
                self.log_file = logs[selected]
                self.network_manager.get(QNetworkRequest(QUrl("http://{}:{}/{}".format(self.host, self.port, self.log_file))))
                return
        QApplication.instance().quit();


    def save_remote_log(self, reply):
        brewery_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
        log_dir = os.path.join(brewery_root_path, "remote_logs")
        if not os.path.exists(log_dir):
            os.mkdir(log_dir)
        self.log_file = os.path.join(log_dir, os.path.basename(self.log_file))
        remote_log = file(self.log_file, "w")
        while reply.canReadLine():
            remote_log.write(reply.readLine())
        remote_log.close()
        self.load_log()

