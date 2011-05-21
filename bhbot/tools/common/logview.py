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

# Tango Colors (http://colors.bravo9.com/tango/)
# Used  Color   Name
#  X   #eeeeec Aluminium 1
#  x   #d3d7cf Aluminium 2
#  X   #babdb6 Aluminium 3
#  X   #888a85 Aluminium 4
#  X   #555753 Aluminium 5
#  X   #2e3436 Aluminium 6
#  X   #fce94f Butter 1
#  X   #edd400 Butter 2
#  X   #c4a000 Butter 3
#  X   #8ae234 Chameleon 1
#  X   #73d216 Chameleon 2
#  X   #4e9a06 Chameleon 3
#  X   #e9b96e Chocolate 1
#  X   #c17d11 Chocolate 2
#  X   #8f5902 Chocolate 3
#  X   #fcaf3e Orange 1
#  X   #f57900 Orange 2
#  X   #ce5c00 Orange 3
#  X   #ad7fa8 Plum 1
#  X   #75507b Plum 2
#  X   #5c3566 Plum 3
#  X   #ef2929 Scarlet Red 1
#  X   #cc0000 Scarlet Red 2
#  X   #a40000 Scarlet Red 3
#  X   #729fcf Sky Blue 1
#  X   #3465a4 Sky Blue 2
#  X   #204a87 Sky Blue 3

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
              ["StorePiece1"           , "Store Piece 1"          , "#ef2929", True],
              ["StorePiece2"           , "Store Piece 2"          , "#2e3436", True],
              ["StorePiece3"           , "Store Piece 3"          , "#8ae234", True],
              ["ReleasePiece"          , "Release Piece"          , "#fcaf3e", True],
              ["OpenNippers"           , "Open Nippers"           , "#eeeeec", True],
              ["CloseNippers"          , "Close Nippers"          , "#d3d7cf", True],
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

    COLUMN_NUMBER = 0
    COLUMN_TIME = 1
    COLUMN_SENDER = 2
    COLUMN_TYPE = 3
    COLUMN_STATE = 4
    COLUMN_CONTENT = 5
    COLUMN_COUNT = 6

    def __init__(self, log, categories, parent = None):
        QAbstractTableModel.__init__(self, parent)
        self.log = []
        self.filtered_log = []
        self.load_log(log)
        self.filter(categories)


    def rowCount(self, parent):
        return len(self.filtered_log)


    def columnCount(self, parent):
        return LogModel.COLUMN_COUNT


    def data(self, index, role):
        global CATEGORIES
        global CATEGORIES_COLORS

        if role == Qt.DisplayRole:
            if index.column() == LogModel.COLUMN_TYPE:
                return self.filtered_log[index.row()][index.column()][1]
            else:
                return self.filtered_log[index.row()][index.column()]
        elif role == Qt.DecorationRole:
            if index.column() == LogModel.COLUMN_TYPE:
                return QColor(self.filtered_log[index.row()][index.column()][2])
            else:
                return QVariant()
        else:
            return QVariant()


    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if section == LogModel.COLUMN_NUMBER:
                return "#"
            elif section == LogModel.COLUMN_TIME:
                return "Time"
            elif section == LogModel.COLUMN_SENDER:
                return "Sender"
            elif section == LogModel.COLUMN_TYPE:
                return "Type"
            elif section == LogModel.COLUMN_STATE:
                return "State"
            elif section == LogModel.COLUMN_CONTENT:
                return "Content"
        else:
            return QVariant()


    def load_log(self, log):
        global CATEGORIES
        index = 0
        current_state = "None"
        for logline in log:
            if len(logline) == 2:
                category_index = self.get_category_index('str')
                category_data = CATEGORIES[category_index]
                text = logline[1][2:]
                self.log.append([index, logline[0], "", category_data, current_state, text, category_index])
                if text.startswith("Switching to"):
                    current_state = text[text.rfind(" ") + 1:]
            else:
                category_index = self.get_category_index(logline[1])
                category_data = CATEGORIES[category_index]

                self.log.append([index, logline[0], logline[2], category_data, current_state, str(helpers.translate_packet_data(logline[1], logline[3]))[1:-1], category_index])
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
            if logline[6] in categories:
                self.filtered_log.append(logline)
        self.reset()




class TrajectoryScene(QGraphicsScene):

    def __init__(self, log):
        QGraphicsScene.__init__(self)
        self.setBackgroundBrush(QBrush(QColor("#82a2cf")))

        self.field = QGraphicsSvgItem(os.path.join(os.path.dirname(os.path.dirname(__file__)), "simulator", "field.svg"))
        self.addItem(self.field)
        self.field.setPos(-102.0, -102.0)

        pen = QPen(QColor("#2e3436"), 10)
        font = QFont()
        font.setPointSize(70)
        self.addLine(-170.0, -170.0, -170.0, 100.0, pen)
        self.addText("x", font).setPos(-200.0, 80.0)
        self.addLine(-170.0, -170.0, 100.0, -170.0, pen)
        self.addText("y", font).setPos(120.0, -230.0)

        trajPath = QPainterPath()
        expectedPath = QPainterPath()
        firstGoto = True
        startX = None
        startY = None
        for logline in log:
            if len(logline) != 2:
                if logline[1] == "KeepAlive":
                    coords = logline[3]['current_pose']
                    x = coords[1] * 1000
                    y = coords[0] * 1000
                    if firstGoto:
                        trajPath.moveTo(x, y)
                        startX = x
                        startY = y
                    else:
                        trajPath.lineTo(x, y)
                elif logline[1] == "Goto":
                    if logline[3]['movement'] != 'MOVEMENT_ROTATE':
                        points = logline[3]['points']
                        for coords in points:
                            x = coords[1] * 1000
                            y = coords[0] * 1000
                            if firstGoto:
                                firstGoto = False
                                expectedPath.moveTo(startX, startY)
                            expectedPath.lineTo(x, y)

        self.trajItem = QGraphicsPathItem(trajPath)
        self.trajItem.setPen(QPen(QColor("#edd400"), 10))
        self.addItem(self.trajItem)

        self.expectedItem = QGraphicsPathItem(expectedPath)
        self.expectedItem.setPen(QPen(QColor("#73d216"), 10))
        self.addItem(self.expectedItem)

        # Display world
        #worldPen = QPen(QColor("#8ae234"), 10)
        #for edge in world.world.edges:
            #self.addLine(edge.node1.y, edge.node1.x, edge.node2.y, edge.node2.x, worldPen)
        #worldPen = QPen(QColor("#4e9a06"), 10)
        #worldBrush = QBrush(QColor("#8ae234"))
        #for node in world.world.nodes:
            #self.addEllipse(node.y - 20.0, node.x - 20.0, 40.0, 40.0, worldPen, worldBrush)




class TrajectoryView(QGraphicsView):

    def __init__(self, parent = None):
        QGraphicsView.__init__(self, parent)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)

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
        self.traj_scene = TrajectoryScene(log)
        trajectory_view.setScene(self.traj_scene)
        layout = QBoxLayout(QBoxLayout.LeftToRight)
        layout.setSpacing(0.0)
        layout.setContentsMargins(0.0, 0.0, 0.0, 0.0)
        self.ui.field_container.setLayout(layout)
        layout.addWidget(trajectory_view)

        self.ui.expected_checkbox.stateChanged.connect(self.expected_changed)
        self.traj_scene.expectedItem.setVisible(self.ui.expected_checkbox.isChecked())
        self.ui.real_checkbox.stateChanged.connect(self.real_changed)
        self.traj_scene.trajItem.setVisible(self.ui.real_checkbox.isChecked())


    def expected_changed(self, state):
        self.traj_scene.expectedItem.setVisible(self.ui.expected_checkbox.isChecked())


    def real_changed(self, state):
        self.traj_scene.trajItem.setVisible(self.ui.real_checkbox.isChecked())


    def load_log(self, filepath):
        logcontent = imp.load_source("logcontent", filepath)
        return logcontent.log



    def updateView(self, selected, deselected):
        cats = []
        for index in self.ui.categories.selectionModel().selection().indexes():
            cats.append(index.row())
        self.ui.log_view.model().filter(cats)
