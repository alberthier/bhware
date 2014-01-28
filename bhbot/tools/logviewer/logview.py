# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
import collections

import packets
import logger
from definitions import *

import helpers




class CategoriesModel(QStandardItemModel):

    PACKET_TYPE_ROLE = Qt.UserRole + 1

    def __init__(self, parent = None):
        QStandardItemModel.__init__(self, parent)
        self.setHorizontalHeaderLabels(["Categories"])
        icon_size = QSize(16, 16)
        row = 0
        for packet in packets.PACKETS_LIST:
            pixmap = QPixmap(icon_size)
            pixmap.fill(QColor(packets.COLORS[packet.TYPE]))
            item = QStandardItem(QIcon(pixmap), packet.__name__)
            item.setData(packet.TYPE, CategoriesModel.PACKET_TYPE_ROLE)
            self.appendRow([item])
            row += 1
        pixmap = QPixmap(icon_size)
        pixmap.fill(QColor("#a9a9a9"))
        item = QStandardItem(QIcon(pixmap), "LogText")
        item.setData(-1, CategoriesModel.PACKET_TYPE_ROLE)
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
        for type, packet in packets.PACKETS_BY_TYPE.items():
            pixmap = QPixmap(icon_size)
            pixmap.fill(QColor(packets.COLORS[packet.TYPE]))
            self.colors[type] = QIcon(pixmap)
        pixmap = QPixmap(icon_size)
        pixmap.fill(QColor("#a9a9a9"))
        self.comment_color = QIcon(pixmap)
        self.states_stack = collections.deque(["<None>"])


    def process_log_line(self, log_line, lineno, last_lineno):
        packet_type = log_line[logger.LOG_LINE_PACKET]
        line = []
        first_item = QStandardItem(str(lineno))
        line.append(first_item)
        line.append(QStandardItem(log_line[logger.LOG_LINE_TIME]))
        line.append(QStandardItem(log_line[logger.LOG_LINE_SENDER]))
        if type(packet_type) is str:
            typeItem = QStandardItem(self.comment_color, packet_type)
            typeItem.setData(-1, LogModel.LOG_MODEL_PACKET_TYPE_ROLE)
        else:
            typeItem = QStandardItem(self.colors[packet_type.TYPE], packet_type.__name__)
            typeItem.setData(packet_type.TYPE, LogModel.LOG_MODEL_PACKET_TYPE_ROLE)
        line.append(typeItem)
        line.append(QStandardItem(self.states_stack[-1]))
        line.append(QStandardItem(str(log_line[logger.LOG_LINE_CONTENT + 1])))
        self.appendRow(line)
        if type(packet_type) is str:
            content = log_line[logger.LOG_LINE_CONTENT]
            if content.startswith("# Pushing sub-state "):
                self.states_stack.append(content[content.rfind(" ") + 1:])
            elif content.startswith("# Poping sub-state "):
                self.states_stack.pop()
            elif content.startswith("# Switching to state "):
                self.states_stack[-1] = content[content.rfind(" ") + 1:]




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
        t = index.data(LogModel.LOG_MODEL_PACKET_TYPE_ROLE)
        return t in self.displayed_items




class LogViewController(QObject):

    def __init__(self, ui):
        QObject.__init__(self)

        self.ui = ui

        categories_model = CategoriesModel(self)
        self.ui.categories.setModel(categories_model)
        selection = QItemSelection()
        for row in range(categories_model.rowCount()):
            idx = categories_model.index(row, 0)
            packet_type = idx.data(CategoriesModel.PACKET_TYPE_ROLE)
            if packet_type in packets.PACKETS_BY_TYPE:
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


    def log_loaded(self):
        pass


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
        for row in range(self.ui.log_view.model().rowCount(index)):
            child_index = index.child(row, 0)
            self.expand_packet(child_index, expand)
