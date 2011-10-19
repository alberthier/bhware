#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
import collections

import packets
import logger

from definitions import *




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
            row += 1
        pixmap = QPixmap(icon_size)
        pixmap.fill(QColor("#a9a9a9"))
        item = QStandardItem(QIcon(pixmap), "LogText")
        item.setData(QVariant(-1), CategoriesModel.PACKET_TYPE_ROLE)
        self.appendRow([item])




class LogModel(QStandardItemModel):

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


    def process_log_line(self, log_line, lineno, last_lineno):
        line = []
        first_item = QStandardItem(0)
        line.append(first_item)
        line.append(QStandardItem(log_line[logger.LOG_DATA_TIME]))
        packet_type = log_line[logger.LOG_DATA_TYPE]
        if not packets.PACKETS_BY_NAME.has_key(packet_type):
            line.append(QStandardItem())
            typeItem = QStandardItem(self.comment_color, "Log Text")
            typeItem.setData(QVariant(-1), LogModel.LOG_MODEL_PACKET_TYPE_ROLE)
            line.append(typeItem)
            line.append(QStandardItem("CurrentState"))
            line.append(QStandardItem(packet_type))
        else:
            packet = log_line[logger.LOG_DATA_PACKET]
            line.append(QStandardItem(log_line[logger.LOG_DATA_SENDER]))
            typeItem = QStandardItem(self.colors[packet.TYPE], packet_type)
            typeItem.setData(QVariant(packet.TYPE), LogModel.LOG_MODEL_PACKET_TYPE_ROLE)
            line.append(typeItem)
            line.append(QStandardItem("CurrentState"))
            packet_dict = packet.to_dict(True)
            line.append(QStandardItem(self.to_text(packet_dict)))
            self.add_content(first_item, None, packet_dict)
        self.appendRow(line)


    def add_content(self, parent_item, name, log, indent = ""):
        new_parent = parent_item
        if name != None:
            line = []
            first_item = QStandardItem()
            line.append(first_item)
            line.append(QStandardItem())
            line.append(QStandardItem())
            line.append(QStandardItem())
            line.append(QStandardItem(indent + name))
            line.append(QStandardItem(self.to_text(log)))
            parent_item.appendRow(line)
            new_parent = first_item
        if isinstance(log, list) or isinstance(log, tuple):
            index = 0
            for value in log:
                self.add_content(new_parent, "[{0}]".format(index), value, indent + "    ")
                index += 1
        elif isinstance(log, collections.OrderedDict):
            for key, value in log.iteritems():
                self.add_content(new_parent, key, value, indent + "    ")


    def to_text(self, log):
        if isinstance(log, list) or isinstance(log, tuple):
            text = "["
            first = True
            for value in log:
                if first:
                    first = False
                else:
                    text += ", "
                text += self.to_text(value)
            text += "]"
        elif isinstance(log, collections.OrderedDict) or isinstance(log, dict):
            text = "{"
            first = True
            for key, value in log.iteritems():
                if first:
                    first = False
                else:
                    text += ", "
                text += self.to_text(key) + ": " + self.to_text(value)
            text += "}"
        else:
            text = str(log)
        return text





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




class LogViewController(QObject):

    def __init__(self, ui):
        QObject.__init__(self)

        self.ui = ui

        categories_model = CategoriesModel(self)
        self.ui.categories.setModel(categories_model)
        selection = QItemSelection()
        for row in xrange(categories_model.rowCount()):
            idx = categories_model.index(row, 0)
            packet_type = idx.data(CategoriesModel.PACKET_TYPE_ROLE).toInt()[0]
            if packets.PACKETS_BY_TYPE.has_key(packet_type):
                if packets.PACKETS_BY_TYPE[packet_type].LOGVIEW_DEFAULT_ENABLED:
                    selection.select(idx, idx)
            else:
                selection.select(idx, idx)
        selection_model = self.ui.categories.selectionModel()
        selection_model.select(selection, QItemSelectionModel.Select)
        selection_model.selectionChanged.connect(self.update_view)

        log_model = LogModel()
        filter_model = LogFilterProxyModel()
        filter_model.setSourceModel(log_model)
        self.ui.log_view.setModel(filter_model)
        self.ui.log_view.doubleClicked.connect(self.packet_double_clicked)
        self.ui.log_view.header().setResizeMode(QHeaderView.ResizeToContents)


    def process_log_line(self, log_line, lineno, last_lineno):
        self.ui.log_view.model().sourceModel().process_log_line(log_line, lineno, last_lineno)
        if lineno == last_lineno:
            self.update_view(None, None)


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
