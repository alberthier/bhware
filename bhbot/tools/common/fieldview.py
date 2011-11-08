#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *
import os

import helpers
from definitions import *




class FieldScene(QGraphicsScene):

    def __init__(self, parent = None):
        QGraphicsScene.__init__(self, parent)
        self.setBackgroundBrush(QBrush(QColor("#82a2cf")))
        self.root = QGraphicsItemGroup()
        self.addItem(self.root)
        field = QGraphicsSvgItem(os.path.join(os.path.dirname(__file__), "field.svg"))
        self.root.addToGroup(field)
        field.setPos(-102.0, -102.0)

        pen = QPen(QColor("#2e3436"), 10)
        pen.setCapStyle(Qt.RoundCap)
        font = QFont()
        font.setPointSize(70)
        self.addLine(-170.0, -170.0, -170.0, 100.0, pen)
        self.addLine(-170.0, 100.0, -150.0, 70.0, pen)
        self.addLine(-170.0, 100.0, -190.0, 70.0, pen)
        self.addText("x", font).setPos(-200.0, 80.0)
        self.addLine(-170.0, -170.0, 100.0, -170.0, pen)
        self.addLine(100.0, -170.0, 70.0, -190.0, pen)
        self.addLine(100.0, -170.0, 70.0, -150.0, pen)
        self.addText("y", font).setPos(120.0, -230.0)

        self.layers = []

        self.mouseMoveEventListeners = []
        self.wheelEventListeners = []


    def add_layer(self, layer):
        layer.setParentItem(self.root)
        if len(self.layers) != 0:
            layer.stackBefore(self.layers[-1])
        self.layers.append(layer)


    def mouseMoveEvent(self, event):
        for listener in self.mouseMoveEventListeners:
            listener.sceneMouseMoveEvent(event)


    def wheelEvent(self, event):
        for listener in self.wheelEventListeners:
            listener.sceneWheelEvent(event)




class Layer(QGraphicsItemGroup):

    def __init__(self, parent = None):
        QGraphicsItemGroup.__init__(self)
        self.name = "Unnamed"
        self.color = "#000000"




class GhostRobotLayer(Layer):

    def __init__(self, parent = None):
        Layer.__init__(self, parent)

        self.name = "Ghost robot"
        self.color = "#ffdab9"

        # Position indication
        font = QFont()
        font.setPointSize(50)
        self.x_label = QGraphicsSimpleTextItem()
        self.x_label.setPos(700.0, -200.0)
        self.x_label.setFont(font)
        self.addToGroup(self.x_label)
        self.y_label = QGraphicsSimpleTextItem()
        self.y_label.setPos(1100.0, -200.0)
        self.y_label.setFont(font)
        self.addToGroup(self.y_label)
        self.angle_label = QGraphicsSimpleTextItem()
        self.angle_label.setPos(1500.0, -200.0)
        self.angle_label.setFont(font)
        self.addToGroup(self.angle_label)

        # Ghost robot
        ghost_pen = QPen(QColor(self.color))
        self.mouse_item = helpers.create_robot_base_item(ghost_pen, QBrush(), ghost_pen)

        line = QGraphicsLineItem(-50.0, 0.0, 50.0, 0.0)
        line.setPen(ghost_pen)
        self.mouse_item.addToGroup(line)

        line = QGraphicsLineItem(0.0, -50.0, 0.0, 50.0)
        line.setPen(ghost_pen)
        self.mouse_item.addToGroup(line)

        self.mouse_item.setVisible(False)
        self.addToGroup(self.mouse_item)


    def sceneEnterEvent(self, event):
        self.mouse_item.setVisible(True)


    def sceneLeaveEvent(self, event):
        self.mouse_item.setVisible(False)


    def sceneMouseMoveEvent(self, event):
        pos = event.scenePos()
        self.mouse_item.setPos(pos)
        self.x_label.setText("x = {0:=0.04f}".format(pos.y() / 1000.0))
        self.y_label.setText("y = {0:=0.04f}".format(pos.x() / 1000.0))
        angle = self.convert_angle()
        self.angle_label.setText("angle = {0:=0.04f} ({1:=0.01f} deg)".format(angle, angle / math.pi * 180.0))


    def sceneWheelEvent(self, event):
        angle = (self.mouse_item.rotation() + event.delta() / 8) % 360.0
        self.mouse_item.setRotation(angle)
        angle = self.convert_angle()
        self.angle_label.setText("angle = {0:=0.04f} ({1:=0.01f} deg)".format(angle, angle / math.pi * 180.0))


    def convert_angle(self):
        angle = (self.mouse_item.rotation() % 360.0) / 180.0 * math.pi
        return math.atan2(math.cos(angle), math.sin(angle))




class FieldView(QGraphicsView):

    def __init__(self, parent = None):
        QGraphicsView.__init__(self, parent)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setSceneRect(-200.0, -200.0, 3404, 2504)

        self.enterEventListeners = []
        self.leaveEventListeners = []


    def resizeEvent(self, event):
        QGraphicsView.resizeEvent(self, event)
        self.fitInView(-200.0, -200.0, 3404, 2504, Qt.KeepAspectRatio)


    def enterEvent(self, event):
        for listener in self.enterEventListeners:
            listener.sceneEnterEvent(event)


    def leaveEvent(self, event):
        for listener in self.leaveEventListeners:
            listener.sceneLeaveEvent(event)




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

        ghost_layer = GhostRobotLayer()
        field_view.enterEventListeners.append(ghost_layer)
        field_view.leaveEventListeners.append(ghost_layer)
        self.field_scene.mouseMoveEventListeners.append(ghost_layer)
        self.field_scene.wheelEventListeners.append(ghost_layer)
        self.field_scene.add_layer(ghost_layer)


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
