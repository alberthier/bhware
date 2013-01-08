# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *



class Graph(QGraphicsItemGroup):

    def __init__(self):
        QGraphicsItemGroup.__init__(self)
        #self.addItem(QGraphicsLineItem(0.0, self.get_y(-0.1),

    def get_y(self, value):
        return 100.0 - (value * 100.0)




class SpeedViewController(QObject):

    def __init__(self, ui, parent = None):
        QObject.__init__(self)

        self.scene = QGraphicsScene()
        self.graph = Graph()
        self.scene.addItem(self.graph)


    def process_log_line(self, log_line, lineno, last_lineno):
        pass
