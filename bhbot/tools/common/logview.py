#!/usr/bin/env python
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *
from PyQt4 import uic
import os
import sys
import imp
import collections

import helpers
import world
import packets

from definitions import *

# Mozilla Colors (http://colors.bravo9.com/mozilla/)
# Used  Color   Name                        Used  Color   Name
#      #f0f8ff aliceblue                         #ffa07a lightsalmon
#      #faebd7 antiquewhite                      #20b2aa lightseagreen
#      #00ffff aqua                              #87cefa lightskyblue
#      #7fffd4 aquamarine                        #778899 lightslategray
#      #f0ffff azure                             #778899 lightslategrey
#      #f5f5dc beige                             #ffffe0 lightyellow
#      #ffe4c4 bisque                            #00ff00 lime
#      #000000 black                             #32cd32 limegreen
#      #0000ff blue                              #faf0e6 linen
#   X  #8a2be2 blueviolet                        #ff00ff magenta
#   X  #a52a2a brown                             #800000 maroon
#   X  #deb887 burlywood                         #66cdaa mediumaquamarine
#   X  #5f9ea0 cadetblue                         #0000cd mediumblue
#   X  #7fff00 chartreuse                        #ba55d3 mediumorchid
#   X  #d2691e chocolate                         #9370db mediumpurple
#   X  #ff7f50 coral                             #3cb371 mediumseagreen
#   X  #6495ed cornflowerblue                    #7b68ee mediumslateblue
#      #fff8dc cornsilk                          #00fa9a mediumspringgreen
#   X  #dc143c crimson                           #48d1cc mediumturquoise
#   X  #00ffff cyan                              #c71585 mediumvioletred
#   X  #00008b darkblue                          #191970 midnightblue
#   X  #008b8b darkcyan                          #f5fffa mintcream
#   X  #b8860b darkgoldenrod                     #ffe4e1 mistyrose
#   X  #a9a9a9 darkgray                          #ffe4b5 moccasin
#   X  #006400 darkgreen                         #ffdead navajowhite
#   X  #a9a9a9 darkgrey                          #000080 navy
#   X  #bdb76b darkkhaki                         #fdf5e6 oldlace
#   X  #8b008b darkmagenta                       #808000 olive
#   X  #556b2f darkolivegreen                    #6b8e23 olivedrab
#   X  #ff8c00 darkorange                        #ffa500 orange
#   X  #9932cc darkorchid                        #ff4500 orangered
#   X  #8b0000 darkred                           #da70d6 orchid
#   X  #e9967a darksalmon                        #eee8aa palegoldenrod
#   X  #8fbc8f darkseagreen                      #98fb98 palegreen
#   X  #483d8b darkslateblue                     #afeeee paleturquoise
#   X  #2f4f4f darkslategray                     #db7093 palevioletred
#   X  #2f4f4f darkslategrey                     #ffefd5 papayawhip
#      #9400d3 darkviolet                        #ffdab9 peachpuff
#   X  #ff1493 deeppink                          #cd853f peru
#      #00bfff deepskyblue                       #ffc0cb pink
#      #696969 dimgray                           #dda0dd plum
#      #696969 dimgrey                           #b0e0e6 powderblue
#      #1e90ff dodgerblue                        #800080 purple
#      #b22222 firebrick                         #ff0000 red
#      #fffaf0 floralwhite                       #bc8f8f rosybrown
#      #228b22 forestgreen                       #4169e1 royalblue
#      #ff00ff fuchsia                           #8b4513 saddlebrown
#      #dcdcdc gainsboro                         #fa8072 salmon
#      #f8f8ff ghostwhite                        #f4a460 sandybrown
#   X  #ffd700 gold                           X  #2e8b57 seagreen
#   X  #daa520 goldenrod                         #fff5ee seashell
#      #808080 gray                              #a0522d sienna
#      #008000 green                             #c0c0c0 silver
#      #adff2f greenyellow                       #87ceeb skyblue
#      #808080 grey                              #6a5acd slateblue
#      #f0fff0 honeydew                          #708090 slategray
#      #ff69b4 hotpink                           #708090 slategrey
#      #cd5c5c indianred                         #fffafa snow
#      #4b0082 indigo                         X  #00ff7f springgreen
#      #fffff0 ivory                          X  #4682b4 steelblue
#      #f0e68c khaki                             #d2b48c tan
#      #e6e6fa lavender                          #008080 teal
#      #fff0f5 lavenderblush                     #d8bfd8 thistle
#      #7cfc00 lawngreen                         #ff6347 tomato
#      #fffacd lemonchiffon                      #40e0d0 turquoise
#      #add8e6 lightblue                         #ee82ee violet
#      #f08080 lightcoral                        #f5deb3 wheat
#      #e0ffff lightcyan                         #ffffff white
#      #fafad2 lightgoldenrodyellow              #f5f5f5 whitesmoke
#      #90ee90 lightgreen                        #ffff00 yellow
#      #d3d3d3 lightgrey                         #9acd32 yellowgreen
#      #ffb6c1 lightpink




class CategoriesModel(QStandardItemModel):

    PACKET_TYPE_ROLE = Qt.UserRole + 1

    def __init__(self, parent = None):
        QStandardItemModel.__init__(self, parent)
        self.setHorizontalHeaderLabels(["Categories"])
        icon_size = QSize(16, 16)
        row = 0
        for packet in packets.PACKETS_LIST:
            pixmap = QPixmap(icon_size)
            pixmap.fill(QColor(packet.LOGVIEW_COLOR))
            item = QStandardItem(QIcon(pixmap), packet.__name__)
            item.setData(QVariant(packet.TYPE), CategoriesModel.PACKET_TYPE_ROLE)
            self.appendRow([item])
            idx = self.index(row, 0)
            row += 1




class LogModel(QStandardItemModel):

    LOG_LINE_TIME   = 0
    LOG_LINE_TYPE   = 1
    LOG_LINE_SENDER = 2
    LOG_LINE_PACKET = 3

    LOG_MODEL_COLUMN_NUMBER  = 0
    LOG_MODEL_COLUMN_TIME    = 1
    LOG_MODEL_COLUMN_SENDER  = 2
    LOG_MODEL_COLUMN_TYPE    = 3
    LOG_MODEL_COLUMN_STATE   = 4
    LOG_MODEL_COLUMN_CONTENT = 5

    LOG_MODEL_PACKET_TYPE_ROLE = Qt.UserRole + 1

    def __init__(self, parent = None):
        QStandardItemModel.__init__(self, parent)
        self.setHorizontalHeaderLabels(["#", "Time", "Sender", "Type", "State", "content"])
        self.colors = {}
        icon_size = QSize(16, 16)
        for type, packet in packets.PACKETS_BY_TYPE.iteritems():
            pixmap = QPixmap(icon_size)
            pixmap.fill(QColor(packet.LOGVIEW_COLOR))
            self.colors[type] = QIcon(pixmap)
        pixmap = QPixmap(icon_size)
        pixmap.fill(QColor("#a9a9a9"))
        self.comment_color = QIcon(pixmap)


    def load_log(self, log):
        for line in log:
            self.add_log_line(line)


    def add_log_line(self, log_line):
        line = []
        first_item = QStandardItem(0)
        line.append(first_item)
        line.append(QStandardItem(log_line[LogModel.LOG_LINE_TIME]))
        packet_type = log_line[LogModel.LOG_LINE_TYPE]
        if not packets.PACKETS_BY_NAME.has_key(packet_type):
            line.append(QStandardItem())
            typeItem = QStandardItem(self.comment_color, "Log Text")
            typeItem.setData(QVariant(-1), LogModel.LOG_MODEL_PACKET_TYPE_ROLE)
            line.append(typeItem)
            line.append(QStandardItem("CurrentState"))
            line.append(QStandardItem(packet_type))
        else:
            packet = packets.PACKETS_BY_NAME[packet_type]()
            packet.from_dict(log_line[LogModel.LOG_LINE_PACKET])
            line.append(QStandardItem(log_line[LogModel.LOG_LINE_SENDER]))
            typeItem = QStandardItem(self.colors[packet.TYPE], packet_type)
            typeItem.setData(QVariant(packet.TYPE), LogModel.LOG_MODEL_PACKET_TYPE_ROLE)
            line.append(typeItem)
            line.append(QStandardItem("CurrentState"))
            packet_dict = packet.to_dict(True)
            line.append(QStandardItem(str(packet_dict)))
            self.add_content(first_item, packet_dict)
        self.appendRow(line)


    def add_content(self, parent_item, log, indent = ""):
        for name, value in log.iteritems():
            line = []
            first_item = QStandardItem()
            line.append(first_item)
            line.append(QStandardItem())
            line.append(QStandardItem())
            line.append(QStandardItem())
            line.append(QStandardItem(indent + name))
            line.append(QStandardItem(str(value)))
            if isinstance(value, collections.OrderedDict):
                self.add_content(first_item, value, indent + "    ")
            parent_item.appendRow(line)




class LogFilterProxyModel(QSortFilterProxyModel):

    def __init__(self, parent = None):
        QSortFilterProxyModel.__init__(self, parent)
        self.displayed_items = []


    def filterAcceptsRow(self, sourceRow, sourceParent):
        r = sourceRow
        p = sourceParent
        while p.isValid():
            r = p.row()
            p = p.parent()
        index = self.sourceModel().index(r, LogModel.LOG_MODEL_COLUMN_TYPE)
        t = index.data(LogModel.LOG_MODEL_PACKET_TYPE_ROLE).toInt()[0]
        return t in self.displayed_items




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
        #for logline in log:
            #if len(logline) != 2:
                #if logline[1] == "KeepAlive":
                    #coords = logline[3]['current_pose']
                    #x = coords[1] * 1000
                    #y = coords[0] * 1000
                    #if firstGoto:
                        #trajPath.moveTo(x, y)
                        #startX = x
                        #startY = y
                    #else:
                        #trajPath.lineTo(x, y)
                #elif logline[1] == "Goto":
                    #if logline[3]['movement'] != 'MOVEMENT_ROTATE':
                        #points = logline[3]['points']
                        #for coords in points:
                            #x = coords[1] * 1000
                            #y = coords[0] * 1000
                            #if firstGoto:
                                #firstGoto = False
                                #expectedPath.moveTo(startX, startY)
                            #expectedPath.lineTo(x, y)
                #elif logline[1] == "PieceDetected":
                    #packet = logline[3]
                    #if packet["sensor"] == "SENSOR_CENTER":
                    #if packet["sensor"].endswith("BOTTOM"):
                        #start_pose = packet["start_pose"]
                        #end_pose = packet["end_pose"]
                        #x1 = start_pose[1] * 1000
                        #y1 = (start_pose[0] + ROBOT_CENTER_TO_HACKED_SENSOR_DISTANCE) * 1000
                        #x2 = end_pose[1] * 1000
                        #y2 = (end_pose[0] + ROBOT_CENTER_TO_HACKED_SENSOR_DISTANCE) * 1000
                        #print logline[0], math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                        #line = QGraphicsLineItem(x1, y1, x2, y2)
                        #line.setPen(QPen(QColor("#00ff7f"), 20.0))
                        #self.addItem(line)


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

        self.ui.categories.setModel(CategoriesModel(self))

        self.ui.log_view.header().setResizeMode(QHeaderView.ResizeToContents)
        log = self.load_log(log_file)
        log_model = LogModel()
        log_model.load_log(log)
        filter_model = LogFilterProxyModel()
        filter_model.setSourceModel(log_model)
        self.ui.log_view.setModel(filter_model)
        self.ui.log_view.doubleClicked.connect(self.packet_double_clicked)
        self.setup_selection()

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


    def setup_selection(self):
        selection = QItemSelection()
        model = self.ui.categories.model()
        row = 0
        for packet in packets.PACKETS_LIST:
            if packet.LOGVIEW_DEFAULT_ENABLED:
                idx = model.index(row, 0)
                selection.select(idx, idx)
            row += 1
        selection_model = self.ui.categories.selectionModel()
        selection_model.select(selection, QItemSelectionModel.Select)
        selection_model.selectionChanged.connect(self.update_view)
        self.update_view(None, None)


    def expected_changed(self, state):
        self.traj_scene.expectedItem.setVisible(self.ui.expected_checkbox.isChecked())


    def real_changed(self, state):
        self.traj_scene.trajItem.setVisible(self.ui.real_checkbox.isChecked())


    def load_log(self, filepath):
        logcontent = imp.load_source("logcontent", filepath)
        return logcontent.log


    def update_view(self, selected, deselected):
        packet_types = []
        for index in self.ui.categories.selectionModel().selection().indexes():
            packet_types.append(index.data(CategoriesModel.PACKET_TYPE_ROLE))
        filter_model = self.ui.log_view.model()
        filter_model.displayed_items = packet_types
        filter_model.invalidateFilter()


    def packet_double_clicked(self, index):
        index = index.model().index(index.row(), 0)
        if not index.parent().isValid():
            expand = not self.ui.log_view.isExpanded(index)
            self.expand_packet(index, expand)


    def expand_packet(self, index, expand):
        self.ui.log_view.setExpanded(index, expand)
        for row in xrange(self.ui.log_view.model().rowCount(index)):
            child_index = index.child(row, 0)
            self.expand_packet(child_index, expand)
