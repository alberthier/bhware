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
import opponentdetectionview

from definitions import *




class MainWindowController(QObject):

    def __init__(self, log_file, port):
        QObject.__init__(self)

        self.ui = uic.loadUi(os.path.splitext(__file__)[0] + ".ui")
        self.ui.setWindowIcon(QIcon.fromTheme("text-x-generic"))

        self.views = []
        self.views.append(logview.LogViewController(self.ui))
        self.views.append(logfieldview.LogFieldViewController(self.ui))
        self.views.append(speedview.LinearSpeedViewController(self.ui))
        self.views.append(speedview.AngularSpeedViewController(self.ui))
        self.views.append(opponentdetectionview.OpponentDectectionViewController(self.ui))

        self.log_file = log_file
        self.remote_files_to_download = 0
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


    def find_svg_images(self, log_name, files):
        basename = os.path.splitext(log_name)[0]
        res = []
        for f in files:
            base, ext = os.path.splitext(f)
            if ext == ".svg" and base.startswith(basename):
                res.append(f)
        return res


    def load_log(self):
        log = imp.load_source("logcontent", self.log_file).log
        lineno = 1
        last_lineno = len(log)
        for log_line in log:
            content = log_line[logger.LOG_LINE_CONTENT]
            packet_type = log_line[logger.LOG_LINE_PACKET]
            if type(packet_type) != str:
                try :
                    log_line[logger.LOG_LINE_CONTENT] = eval(content)
                except Exception as e :
                    print(e)

            log_line.append(content)
            for view in self.views:
                if packet_type is packets.KeepAlive:
                    match_time = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "match_time")
                    if match_time <= 0 or match_time > 90000:
                        continue
                view.process_log_line(log_line, lineno, last_lineno)
            lineno += 1
        dir_name, log_name = os.path.split(self.log_file)
        if len(dir_name) == 0:
            dir_name = "."
        svg_list = self.find_svg_images(log_name, os.listdir(dir_name))
        cwd = os.getcwd()
        os.chdir(dir_name)
        for svg in svg_list:
            lbl = QLabel()
            lbl.setPixmap(QPixmap(svg))
            self.ui.tabWidget.addTab(lbl, svg)
        os.chdir(cwd)
        for view in self.views:
            view.log_loaded()



    def download_remote_loglist(self):
        self.network_manager = QNetworkAccessManager()
        self.network_manager.finished.connect(self.display_log_chooser)
        url = "http://{}:{}/fsroot/root/bhware/bhbot/logs/".format(self.host, self.port)
        self.network_manager.get(QNetworkRequest(QUrl(url)))


    def display_log_chooser(self, reply):
        logs = []
        images = []
        while reply.canReadLine():
            line = str(reply.readLine(), "utf-8").strip()
            if line.startswith("<tr "):
                idx = line.find("<a href=")
                if idx != -1:
                    idx += 9
                    eidx = line.find("'>", idx)
                    if eidx != -1:
                        fname = line[idx:eidx]
                        if fname.endswith(".py"):
                            logs.append(fname)
                        elif fname.endswith(".jpg") or fname.endswith(".svg"):
                            images.append(fname)
        logs.sort()
        logs = [l for l in reversed(logs)]
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
                log_base = os.path.splitext(self.log_file)[0]
                for img in images:
                    if img.startswith(log_base):
                        self.remote_files_to_download += 1
                        url = "http://{}:{}/fsroot/root/bhware/bhbot/logs/{}".format(self.host, self.port, img)
                        self.network_manager.get(QNetworkRequest(QUrl(url)))
                self.remote_files_to_download += 1
                url = "http://{}:{}/fsroot/root/bhware/bhbot/logs/{}".format(self.host, self.port, self.log_file)
                self.network_manager.get(QNetworkRequest(QUrl(url)))
                return
        QApplication.instance().quit();


    def save_remote_log(self, reply):
        brewery_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
        log_dir = os.path.join(brewery_root_path, "remote_logs")
        if not os.path.exists(log_dir):
            os.mkdir(log_dir)
        log_file = os.path.basename(reply.request().url().path())
        log_path = os.path.join(log_dir, log_file)
        if log_path.endswith(".py"):
            self.log_file = log_path
        remote_log = open(log_path, "wb")
        while True:
            data = reply.read(4096)
            if data != None:
                remote_log.write(data)
            else:
                break
        remote_log.close()
        self.remote_files_to_download -= 1
        if self.remote_files_to_download == 0:
            self.load_log()

