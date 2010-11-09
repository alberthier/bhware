#!/usr/bin/env python
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic
import sys




CATEGORIES = ["Keep Alive", "Controller Ready", "Device Busy", "Device Ready", "Start", "Goto", "Goto Started", "Goto Finished", "Blocked", "Enable Anti-Blocking", "Disable Anti-Blocking", "Position Control Config", "Stop", "Resettle", "Reinitialize", "Simulator Data", "Turret Detect", "Log Text"]
CATEGORIES_COLORS = ["#edd400", "#f57900", "#ce5c00", "#73d216", "#729fcf", "#3465a4", "#ad7fa8", "#8f5902", "#4e9a06", "#a40000", "#fce94f", "#e9b96e", "#cc0000", "#204a87", "#5c3566", "#c4a000", "#babdb6", "#555753"]


class CategoriesModel(QAbstractListModel):

    def __init__(self, parent = None):
        QAbstractListModel.__init__(self, parent)


    def rowCount(self, parent):
        global CATEGORIES
        return len(CATEGORIES)


    def data(self, index, role):
        global CATEGORIES
        global CATEGORIES_COLORS

        if role == Qt.DisplayRole:
            return CATEGORIES[index.row()]
        elif role == Qt.DecorationRole:
            return QColor(CATEGORIES_COLORS[index.row()])
        else:
            return QVariant()


    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            return "Categories"
        else:
            return QVariant()




class LogModel(QAbstractTableModel):

    PACKET_TYPES = ["ControllerReady", "DeviceBusy", "DeviceReady", "Start", "Goto", "GotoStarted", "GotoFinished", "Blocked", "EnableAntiBlocking", "DisableAntiBlocking", "KeepAlive", "PositionControlConfig", "Stop", "Resettle", "Reinitialize", "SimulatorData", "TurretDetect"]

    def __init__(self, parent = None):
        QAbstractTableModel.__init__(self, parent)


    def rowCount(self, parent):
        global CATEGORIES
        return len(CATEGORIES)


    def columnCount(self, parent):
        return 2


    def data(self, index, role):
        global CATEGORIES
        global CATEGORIES_COLORS

        if role == Qt.DisplayRole:
            if index.column() == 0:
                return CATEGORIES[index.row()]
            elif index.column() == 1:
                return "content"
        elif role == Qt.DecorationRole:
            if index.column() == 0:
                return QColor(CATEGORIES_COLORS[index.row()])
            else:
                return QVariant()
        else:
            return QVariant()


    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if section == 0:
                return "Type"
            elif section == 1:
                return "Content"
        else:
            return QVariant()




class MainWindowController(QObject):

    def __init__(self):
        QObject.__init__(self)
        global CATEGORIES
        self.ui = uic.loadUi(__file__[:-3] + ".ui")
        self.ui.show()
        model = CategoriesModel(self)
        self.ui.categories.setModel(model)
        selection = QItemSelection(model.index(1, 0), model.index(len(CATEGORIES) - 1, 0))
        self.ui.categories.selectionModel().select(selection, QItemSelectionModel.Select)
        self.ui.log_view.header().setResizeMode(QHeaderView.ResizeToContents)
        self.ui.log_view.setModel(LogModel(self))


if __name__ == "__main__":
    app = QApplication(sys.argv)

    mw = MainWindowController()

    sys.exit(app.exec_())
