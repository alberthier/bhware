# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *


def get_value(dump, name):
    for key, value in dump:
        if key == name:
            return value
    return None




class PlotWidget(QWidget):

    def __init__(self, parent = None):
        QWidget.__init__(self, parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        matplotlib = None
        try:
            import matplotlib
            v = [i for i in map(int, matplotlib.__version__.split('.'))]
            if not (v[0] >= 1 and v[1] >= 2):
                del matplotlib
                matplotlib = None
            else:
                from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
                from matplotlib.figure import Figure
        except:
            pass

        self.has_matplotlib = not matplotlib is None

        if not self.has_matplotlib:
            error = "You need matplotlib v1.2.0 or newer"
            print(error)
            label = QLabel(self)
            label.setText(error)
            label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            layout.addWidget(label)
        else:
            self.figure = Figure((5.0, 4.0), dpi=100)
            self.figure_canvas = FigureCanvas(self.figure)
            self.figure_canvas.setParent(self)
            layout.addWidget(self.figure_canvas)


    def add_plot(self, **kwargs):
        if not self.has_matplotlib:
            return None

        plot = self.figure.add_subplot(111, **kwargs)
        plot.grid(True)
        return plot
