#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *
import os




class FieldScene(QGraphicsScene):

    def __init__(self, parent = None):
        QGraphicsScene.__init__(self, parent)
        self.setBackgroundBrush(QBrush(QColor("#82a2cf")))
        self.root = QGraphicsItemGroup()
        self.addItem(self.root)
        field = QGraphicsSvgItem(os.path.join(os.path.dirname(os.path.dirname(__file__)), "simulator", "field.svg"))
        self.root.addToGroup(field)
        field.setPos(-102.0, -102.0)

        pen = QPen(QColor("#2e3436"), 10)
        font = QFont()
        font.setPointSize(70)
        self.addLine(-170.0, -170.0, -170.0, 100.0, pen)
        self.addText("x", font).setPos(-200.0, 80.0)
        self.addLine(-170.0, -170.0, 100.0, -170.0, pen)
        self.addText("y", font).setPos(120.0, -230.0)

        self.layers = []


    def add_layer(self, layer):
        self.layers.append(layer)
        layer.setParentItem(self.root)




class Layer(QGraphicsItemGroup):

    def __init__(self, parent = None):
        QGraphicsItemGroup.__init__(self)
        self.name = "Unnamed"




class FieldView(QGraphicsView):

    def __init__(self, parent = None):
        QGraphicsView.__init__(self, parent)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setSceneRect(-200.0, -200.0, 3404, 2504)


    def resizeEvent(self, event):
        QGraphicsView.resizeEvent(self, event)
        self.fitInView(-200.0, -200.0, 3404, 2504, Qt.KeepAspectRatio)




class FieldViewController(QObject):

    LAYERS_MODEL_LAYER_ROLE = Qt.UserRole + 1

    def __init__(self, ui, parent = None):
        QObject.__init__(self, parent)
        self.ui = ui

        layout = QHBoxLayout(self.ui.field_view_container)
        layout.setContentsMargins(0, 0, 0, 0)

        field_view = FieldView(self.ui.field_view_container)
        layout.addWidget(field_view)

        self.field_scene = FieldScene(self)
        field_view.setScene(self.field_scene)


    def update_layers_list(self):
        self.layers_model = QStandardItemModel(self)
        self.layers_model.setHorizontalHeaderLabels(["Layers"])

        icon_size = QSize(16, 16)
        selection = QItemSelection()
        row = 0
        for layer in self.field_scene.layers:
            pixmap = QPixmap(icon_size)
            pixmap.fill(QColor(layer.color))
            item = QStandardItem(QIcon(pixmap), layer.name)
            item.setData(layer, FieldViewController.LAYERS_MODEL_LAYER_ROLE)
            self.layers_model.appendRow([item])

            if layer.isVisible():
                idx = self.layers_model.index(row, 0)
                selection.select(idx, idx)

            row += 1

        self.ui.layers_view.setModel(self.layers_model)

        selection_model = self.ui.layers_view.selectionModel()
        selection_model.select(selection, QItemSelectionModel.Select)
        selection_model.selectionChanged.connect(self.update_view)


    def update_view(self, selected, deselected):
        for index in selected.indexes():
            layer = index.data(FieldViewController.LAYERS_MODEL_LAYER_ROLE).toPyObject()
            layer.setVisible(True)
        for index in deselected.indexes():
            layer = index.data(FieldViewController.LAYERS_MODEL_LAYER_ROLE).toPyObject()
            layer.setVisible(False)
