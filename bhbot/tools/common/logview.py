#!/usr/bin/env python
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *
from PyQt4 import uic
import os
import sys
import imp

import helpers
import world

from definitions import *

#               Type                      Displayed name             Color     Displayed by default
CATEGORIES = [["KeepAlive"             , "Keep Alive"             , "#fce94f", False],
              ["SimulatorData"         , "Simulator Data"         , "#c4a000", False],
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
              ["Deployment"            , "Deployment"             , "#75507b", True],
              ["PieceDetected"         , "Piece Detected"         , "#c17d11", True],
              ["StorePiece"            , "Store Piece"            , "#ef2929", True],
              ["PieceStored"           , "Piece Stored"           , "#d4d7cf", True],
              ["ReleasePiece"          , "Release Piece"          , "#fcaf3e", True],
              ["Reinitialize"          , "Reinitialize"           , "#5c3566", True],
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

    def __init__(self, log, categories, parent = None):
        QAbstractTableModel.__init__(self, parent)
        self.log = []
        self.filtered_log = []
        self.load_log(log)
        self.filter(categories)


    def rowCount(self, parent):
        return len(self.filtered_log)


    def columnCount(self, parent):
        return 5


    def data(self, index, role):
        global CATEGORIES
        global CATEGORIES_COLORS

        if role == Qt.DisplayRole:
            if index.column() == 3:
                return self.filtered_log[index.row()][index.column()][1]
            else:
                return self.filtered_log[index.row()][index.column()]
        elif role == Qt.DecorationRole:
            if index.column() == 3:
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
                return "Time"
            elif section == 2:
                return "Sender"
            elif section == 3:
                return "Type"
            elif section == 4:
                return "Content"
        else:
            return QVariant()


    def load_log(self, log):
        global CATEGORIES
        index = 0
        for logline in log:
            if len(logline) == 2:
                category_index = self.get_category_index('str')
                category_data = CATEGORIES[category_index]
                self.log.append([index, logline[0], "", category_data, logline[1][2:], category_index])
            else:
                category_index = self.get_category_index(logline[1])
                category_data = CATEGORIES[category_index]

                self.log.append([index, logline[0], logline[2], category_data, str(helpers.translate_packet_data(logline[1], logline[3]))[1:-1], category_index])
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
            if logline[5] in categories:
                self.filtered_log.append(logline)
        self.reset()




class TrajectoryScene(QGraphicsScene):

    def __init__(self, log):
        QGraphicsScene.__init__(self)
        gradiant = QLinearGradient(0.5, 0.1, 0.5, 0.9)
        gradiant.setCoordinateMode(QGradient.ObjectBoundingMode)
        gradiant.setColorAt(0.0, QColor("#729fcf"))
        gradiant.setColorAt(1.0, QColor("#eeeeec"))
        self.setBackgroundBrush(gradiant)

        self.field = QGraphicsSvgItem(os.path.join(os.path.dirname(os.path.dirname(__file__)), "simulator", "field.svg"))
        self.addItem(self.field)
        self.field.setPos(-102.0, -102.0)

        painterPath = QPainterPath()
        first = True
        for logline in log:
            if len(logline) != 2 and logline[1] == "KeepAlive":
                coords = logline[3]['current_pose']
                x = coords[1] * 1000
                y = coords[0] * 1000
                angle = coords[2] * 1000
                if first:
                    first = False
                    painterPath.moveTo(x, y)
                else:
                    painterPath.lineTo(x, y)

        pathItem = QGraphicsPathItem(painterPath)
        pathItem.setPen(QPen(QColor("#edd400"), 10))
        self.addItem(pathItem)

        # Display world
        worldPen = QPen(QColor("#8ae234"), 10)
        for edge in world.world.edges:
            self.addLine(edge.node1.y, edge.node1.x, edge.node2.y, edge.node2.x, worldPen)
        worldPen = QPen(QColor("#4e9a06"), 10)
        worldBrush = QBrush(QColor("#8ae234"))
        for node in world.world.nodes:
            self.addEllipse(node.y - 20.0, node.x - 20.0, 40.0, 40.0, worldPen, worldBrush)




class TrajectoryView(QGraphicsView):

    def __init__(self, parent = None):
        QGraphicsView.__init__(self, parent)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setSceneRect(-200.0, -200.0, 3404, 2504)


    def resizeEvent(self, event):
        QGraphicsView.resizeEvent(self, event)
        self.fitInView(-200.0, -200.0, 3404, 2504, Qt.KeepAspectRatio)




class MainWindowController(QObject):

    def __init__(self, log_file):
        QObject.__init__(self)
        global CATEGORIES
        self.ui = uic.loadUi(os.path.splitext(__file__)[0] + ".ui")
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
        log = self.load_log(log_file)
        self.ui.log_view.setModel(LogModel(log, filtered_cats, self))
        trajectory_view = TrajectoryView()
        trajectory_view.setScene(TrajectoryScene(log))
        self.ui.trajectory_tab.layout().addWidget(trajectory_view)


    def load_log(self, filepath):
        logcontent = imp.load_source("logcontent", filepath)
        return logcontent.log



    def updateView(self, selected, deselected):
        cats = []
        for index in self.ui.categories.selectionModel().selection().indexes():
            cats.append(index.row())
        self.ui.log_view.model().filter(cats)
