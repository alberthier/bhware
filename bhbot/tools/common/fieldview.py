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
        field.setPos(-150.0, -100.0)

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

        self.mouse_move_event_listeners = []
        self.wheel_event_listeners = []
        self.mouse_press_event_listeners = []


    def mouseMoveEvent(self, event):
        for listener in self.mouse_move_event_listeners:
            listener.mouse_move_event(event.scenePos(), event)


    def wheelEvent(self, event):
        for listener in self.wheel_event_listeners:
            listener.wheel_event(event.scenePos(), event)


    def mousePressEvent(self, event):
        for listener in self.mouse_press_event_listeners:
            listener.mouse_press_event(event.scenePos(), event)




class FieldView(QGraphicsView):

    def __init__(self, parent = None):
        QGraphicsView.__init__(self, parent)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setSceneRect(-200.0, -250.0, 3404, 2450)
        self.setCursor(Qt.BlankCursor)

        self.enter_event_listeners = []
        self.leave_event_listeners = []
        self.key_press_event_listeners = []


    def resizeEvent(self, event):
        QGraphicsView.resizeEvent(self, event)
        self.fitInView(-200.0, -200.0, 3404, 2504, Qt.KeepAspectRatio)


    def enterEvent(self, event):
        for listener in self.enter_event_listeners:
            listener.enter_event(event)


    def leaveEvent(self, event):
        for listener in self.leave_event_listeners:
            listener.leave_event(event)


    def keyPressEvent(self, event):
        pos = self.mapToScene(self.mapFromGlobal(QCursor.pos()))
        for listener in self.key_press_event_listeners:
            listener.key_press_event(pos, event)




class Layer(QGraphicsItemGroup):

    def __init__(self, field_view_controller, name = "Unnamed", color = "#000000", visible = True):
        QGraphicsItemGroup.__init__(self)
        self.field_view_controller = field_view_controller

        self.model_item = QStandardItem()
        self.model_item.setData(self, FieldViewController.LAYERS_MODEL_LAYER_ROLE)
        field_view_controller.layers_model.appendRow([self.model_item])

        self.update_title(name, color)
        self.setVisible(visible)

        self.setParentItem(field_view_controller.field_scene.root)

        if len(field_view_controller.layers) != 0:
            self.stackBefore(field_view_controller.layers[-1])
        field_view_controller.layers.append(self)

        if self.isVisible():
            idx = field_view_controller.layers_model.index(len(field_view_controller.layers) - 1, 0)
            field_view_controller.ui.layers_view.selectionModel().select(idx, QItemSelectionModel.Select)

        field_view_controller.field_view.enter_event_listeners.append(self)
        field_view_controller.field_view.leave_event_listeners.append(self)
        field_view_controller.field_view.key_press_event_listeners.append(self)
        field_view_controller.field_scene.mouse_move_event_listeners.append(self)
        field_view_controller.field_scene.wheel_event_listeners.append(self)
        field_view_controller.field_scene.mouse_press_event_listeners.append(self)


    def update_title(self, name, color):
        self.name = name
        self.color = color
        icon_size = QSize(16, 16)
        pixmap = QPixmap(icon_size)
        pixmap.fill(QColor(color))
        self.model_item.setText(name)
        self.model_item.setIcon(QIcon(pixmap))


    def setup(self):
        pass


    def enter_event(self, event):
        pass


    def leave_event(self, event):
        pass


    def key_press_event(self, pos, event):
        pass


    def mouse_move_event(self, pos, event):
        pass


    def wheel_event(self, pos, event):
        pass


    def mouse_press_event(self, pos, event):
        pass




class GhostRobotLayer(Layer):

    def __init__(self, field_view_controller, main_robot = True):
        Layer.__init__(self, field_view_controller, "Ghost robot", "#702800")

        # Position indication
        font = QFont()
        font.setPointSize(50)
        self.x_label = QGraphicsSimpleTextItem()
        self.x_label.setPos(700.0, -250.0)
        self.x_label.setFont(font)
        self.addToGroup(self.x_label)
        self.y_label = QGraphicsSimpleTextItem()
        self.y_label.setPos(1100.0, -250.0)
        self.y_label.setFont(font)
        self.addToGroup(self.y_label)
        self.angle_label = QGraphicsSimpleTextItem()
        self.angle_label.setPos(1500.0, -250.0)
        self.angle_label.setFont(font)
        self.addToGroup(self.angle_label)

        self.mouse_item = None
        self.is_main_robot = main_robot
        self.update_mouse_item()


    def update_mouse_item(self, pos = None):
        rotation = 0.0
        if self.mouse_item is not None:
            rotation = self.mouse_item.rotation()
            self.removeFromGroup(self.mouse_item)
            self.scene().removeItem(self.mouse_item)
        # Ghost robot
        ghost_pen = QPen(QColor(self.color))
        if self.is_main_robot:
            (self.mouse_item, robot_item, gyration_item) = helpers.create_main_robot_base_item(ghost_pen, QBrush(), ghost_pen)
        else:
            (self.mouse_item, robot_item, gyration_item) = helpers.create_secondary_robot_base_item(ghost_pen, QBrush(), ghost_pen)

        line = QGraphicsLineItem(-50.0, 0.0, 50.0, 0.0)
        line.setPen(ghost_pen)
        self.mouse_item.addToGroup(line)

        line = QGraphicsLineItem(0.0, -50.0, 0.0, 50.0)
        line.setPen(ghost_pen)
        self.mouse_item.addToGroup(line)

        self.mouse_item.setRotation(rotation)
        if pos is None:
            self.mouse_item.setVisible(False)
        else:
            self.mouse_item.setPos(pos.x(), pos.y())
        self.addToGroup(self.mouse_item)


    def enter_event(self, event):
        self.mouse_item.setVisible(True)


    def leave_event(self, event):
        self.mouse_item.setVisible(False)


    def mouse_move_event(self, pos, event):
        self.mouse_item.setPos(pos.x(), pos.y())
        self.x_label.setText("x = {:=0.04f}".format(pos.y() / 1000.0))
        self.y_label.setText("y = {:=0.04f}".format(pos.x() / 1000.0))
        angle = self.convert_angle()
        self.angle_label.setText("angle = {:=0.04f} ({:=0.01f} deg)".format(angle, angle / math.pi * 180.0))


    def wheel_event(self, pos, event):
        angle = (self.mouse_item.rotation() + event.delta() / 8) % 360.0
        self.mouse_item.setRotation(angle)
        angle = self.convert_angle()
        self.angle_label.setText("angle = {:=0.04f} ({:=0.01f} deg)".format(angle, angle / math.pi * 180.0))


    def mouse_press_event(self, pos, event):
        if event.button() == Qt.RightButton:
            self.is_main_robot = not self.is_main_robot
            self.update_mouse_item(pos)


    def convert_angle(self):
        angle = (self.mouse_item.rotation() % 360.0) / 180.0 * math.pi
        return math.atan2(math.cos(angle), math.sin(angle))




class FieldViewController(QObject):

    LAYERS_MODEL_LAYER_ROLE = Qt.UserRole + 1

    def __init__(self, ui, parent = None):
        QObject.__init__(self, parent)

        self.ui = ui
        self.layers = []

        self.layers_model = QStandardItemModel(self)
        self.layers_model.setHorizontalHeaderLabels(["Layers"])
        self.ui.layers_view.setModel(self.layers_model)
        self.ui.layers_view.selectionModel().selectionChanged.connect(self.update_view)

        self.field_scene = FieldScene()

        self.field_view = FieldView(self.ui.field_view_container)
        self.ui.field_view_container_layout.addWidget(self.field_view)
        self.field_view.setScene(self.field_scene)


    def update_view(self, selected, deselected):
        for index in selected.indexes():
            layer = index.data(FieldViewController.LAYERS_MODEL_LAYER_ROLE)
            layer.setVisible(True)
        for index in deselected.indexes():
            layer = index.data(FieldViewController.LAYERS_MODEL_LAYER_ROLE)
            layer.setVisible(False)
