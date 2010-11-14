#!/usr/bin/env python
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic
import os
import sys
import imp

import last_log

# Update PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery"))

from definitions import *

#               Type                      Displayed name             Color     Displayed by default
CATEGORIES = [["KeepAlive"             , "Keep Alive"             , "#fce94f", False],
              ["ControllerReady"       , "Controller Ready"       , "#edd400", True],
              ["DeviceBusy"            , "Device Busy"            , "#f57900", True],
              ["DeviceReady"           , "Device Ready"           , "#ce5c00", True],
              ["Start"                 , "Start"                  , "#73d216", True],
              ["Goto"                  , "Goto"                   , "#729fcf", True],
              ["GotoStarted"           , "Goto Started"           , "#4e9a06", True],
              ["GotoFinished"          , "Goto Finished"          , "#a40000", True],
              ["Blocked"               , "Blocked"                , "#8f5902", True],
              ["EnableAntiBlocking"    , "Enable Anti-Blocking"   , "#3465a4", True],
              ["DisableAntiBlocking"   , "Disable Anti-Blocking"  , "#ad7fa8", True],
              ["PositionControlConfig" , "Position Control Config", "#e9b96e", True],
              ["Stop"                  , "Stop"                   , "#cc0000", True],
              ["Resettle"              , "Resettle"               , "#204a87", True],
              ["Reinitialize"          , "Reinitialize"           , "#5c3566", True],
              ["SimulatorData"         , "Simulator Data"         , "#c4a000", True],
              ["TurretDetect"          , "Turret Detect"          , "#babdb6", True],
              ["str"                   , "Log Text"               , "#555753", True]]


class CategoriesModel(QAbstractListModel):

    def __init__(self, parent = None):
        QAbstractListModel.__init__(self, parent)


    def rowCount(self, parent):
        global CATEGORIES
        return len(CATEGORIES)


    def data(self, index, role):
        global CATEGORIES

        if role == Qt.DisplayRole:
            return CATEGORIES[index.row()][1]
        elif role == Qt.DecorationRole:
            return QColor(CATEGORIES[index.row()][2])
        else:
            return QVariant()


    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            return "Categories"
        else:
            return QVariant()




class LogModel(QAbstractTableModel):

    def __init__(self, filepath, categories, parent = None):
        QAbstractTableModel.__init__(self, parent)
        self.log = []
        self.filtered_log = []
        self.load_log(filepath)
        self.filter(categories)


    def rowCount(self, parent):
        return len(self.filtered_log)


    def columnCount(self, parent):
        return 4


    def data(self, index, role):
        global CATEGORIES
        global CATEGORIES_COLORS

        if role == Qt.DisplayRole:
            if index.column() == 2:
                return self.filtered_log[index.row()][index.column()][1]
            else:
                return self.filtered_log[index.row()][index.column()]
        elif role == Qt.DecorationRole:
            if index.column() == 2:
                return QColor(self.filtered_log[index.row()][index.column()][2])
            else:
                return QVariant()
        else:
            return QVariant()


    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if section == 0:
                return "#"
            elif section == 1:
                return "Sender"
            elif section == 2:
                return "Type"
            elif section == 3:
                return "Content"
        else:
            return QVariant()


    def load_log(self, filepath):
        global CATEGORIES
        logcontent = imp.load_source("logcontent", filepath)
        index = 0
        for logline in logcontent.log:
            if type(logline) == str:
                category_index = self.get_category_index('str')
                category_data = CATEGORIES[category_index]
                self.log.append([index, "", category_data, logline, category_index])
            elif type(logline) == list:
                category_index = self.get_category_index(logline[0])
                category_data = CATEGORIES[category_index]

                self.log.append([index, logline[1], category_data, format_log_line(logline[0], logline[2]), category_index])
            index += 1


    def get_category_index(self, key):
        global CATEGORIES
        index = 0
        for cat in CATEGORIES:
            if cat[0] == key:
                break
            else:
                index += 1
        return index


    def filter(self, categories):
        self.filtered_log = []
        for logline in self.log:
            if logline[4] in categories:
                self.filtered_log.append(logline)
        self.reset()




class MainWindowController(QObject):

    def __init__(self, log_file):
        QObject.__init__(self)
        global CATEGORIES
        self.ui = uic.loadUi(__file__[:-3] + ".ui")
        self.ui.setWindowIcon(QIcon.fromTheme("text-x-generic"))
        self.ui.show()
        model = CategoriesModel(self)
        self.ui.categories.setModel(model)
        selection = QItemSelection()
        cat_index = 0
        filtered_cats = []
        for cat in CATEGORIES:
            if cat[3]:
                idx = model.index(cat_index, 0)
                selection.select(idx, idx)
                filtered_cats.append(cat_index)
            cat_index += 1

        self.ui.categories.selectionModel().select(selection, QItemSelectionModel.Select)
        self.ui.categories.selectionModel().selectionChanged.connect(self.updateView)
        self.ui.log_view.header().setResizeMode(QHeaderView.ResizeToContents)
        self.ui.log_view.setModel(LogModel(log_file, filtered_cats, self))



    def updateView(self, selected, deselected):
        cats = []
        for index in self.ui.categories.selectionModel().selection().indexes():
            cats.append(index.row())
        self.ui.log_view.model().filter(cats)



def format_log_line(packet_type, packet_data):
    if packet_type == "DeviceReady" or packet_type == "Start":
        if packet_data["team"] == TEAM_BLUE:
            packet_data["team"] = "TEAM_BLUE"
        elif packet_data["team"] == TEAM_RED:
            packet_data["team"] = "TEAM_RED"

    elif packet_type == "Goto":
        if packet_data["movement"] == MOVEMENT_ROTATE:
            packet_data["movement"] = "MOVEMENT_ROTATE"
        elif packet_data["movement"] == MOVEMENT_MOVE:
            packet_data["movement"] = "MOVEMENT_MOVE"

        if packet_data["direction"] == DIRECTION_FORWARD:
            packet_data["direction"] = "DIRECTION_FORWARD"
        elif packet_data["direction"] == DIRECTION_BACKWARD:
            packet_data["direction"] = "DIRECTION_BACKWARD"

    elif packet_type == "GotoFinished":
        if packet_data["reason"] == REASON_DESTINATION_REACHED:
            packet_data["reason"] = "REASON_DESTINATION_REACHED"
        elif packet_data["reason"] == REASON_POWN_FOUND:
            packet_data["reason"] = "REASON_POWN_FOUND"
        elif packet_data["reason"] == REASON_QWEEN_FOUND:
            packet_data["reason"] = "REASON_QWEEN_FOUND"
        elif packet_data["reason"] == REASON_KING_FOUND:
            packet_data["reason"] = "REASON_KING_FOUND"

    elif packet_type == "Blocked":
        if packet_data["side"] == BLOCKED_FRONT:
            packet_data["side"] = "BLOCKED_FRONT"
        elif packet_data["side"] == BLOCKED_BACK:
            packet_data["side"] = "BLOCKED_BACK"

    elif packet_type == "Resettle":
        if packet_data["axis"] == AXIS_ABSCISSA:
            packet_data["axis"] = "AXIS_ABSCISSA"
        elif packet_data["axis"] == AXIS_ORDINATE:
            packet_data["axis"] = "AXIS_ORDINATE"

    return str(packet_data)[1:-1]



if __name__ == "__main__":
    app = QApplication(sys.argv)

    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        log_file = last_log.get_last_logfile()

    mw = MainWindowController(log_file)

    sys.exit(app.exec_())
