from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *


class Draw(QWidget):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.polygons = []
        self.pol = QPolygonF()
        self.polygons.append(self.pol)
        self.q = QPointF(-100, -100)
        self.add_vertex = True
        self.greenPol = QPolygonF()
        self.paint_red = False

    def mousePressEvent(self, e: QMouseEvent):

        self.greenPol = QPolygonF()
        # Get cursor position
        x = e.position().x()
        y = e.position().y()

        # Draw polygon
        if self.add_vertex:
            # Create new point
            p = QPointF(x, y)

            # Add point to polygon
            self.pol.append(p)
            self.polygons[0] = self.pol


        # Draw point
        else:
            self.q.setX(x)
            self.q.setY(y)

        # Repaint screen
        self.repaint()

    def paintEvent(self, e: QPaintEvent):
        # Draw situation

        # Create new object
        qp = QPainter(self)

        # Start drawing
        qp.begin(self)

        # Set atributes
        qp.setPen(Qt.GlobalColor.black)
        if self.paint_red:  # Check the flag here
            qp.setBrush(Qt.GlobalColor.red)
        else:
            qp.setBrush(Qt.GlobalColor.yellow)

        # Draw polygon
        for i in self.polygons:
            qp.drawPolygon(i)

        qp.setPen(Qt.GlobalColor.black)
        qp.setBrush(Qt.GlobalColor.green)

        qp.drawPolygon(self.greenPol)

        # Set attributes
        qp.setPen(QPen(Qt.GlobalColor.blue, 4))

        # Draw "+"
        length = 10
        qp.drawLine(int(self.q.x()), int(self.q.y() - length), int(self.q.x()),
                    int(self.q.y() + length))  # Vertical line
        qp.drawLine(int(self.q.x() - length), int(self.q.y()), int(self.q.x() + length),
                    int(self.q.y()))  # Horizontal line

        # End drawing
        qp.end()

    def switchDraw(self):
        # Set the flag to the opposite value when switchDraw is called
        self.add_vertex = not self.add_vertex

    def switch2Point(self):
        # Set the flag to False when switch2Point is called
        self.add_vertex = False

    def switch2Pols(self):
        # Set the flag to True when switch2Pols is called
        self.add_vertex = True

    # return point
    def getPoint(self):
        # Return the point to be analyzed
        return self.q

    # Return polygon
    def getPolygon(self):
        # Return the polygons to be analyzed
        return self.polygons

    def clearData(self):
        # Set the flag to False when clearData is called
        self.switchYellow()
        # Clear point
        self.q.setX(-1000)
        self.q.setY(-1000)

        # Clear polygon
        self.pol.clear()
        self.greenPol.clear()
        self.polygons = [QPolygonF()]

        # Repaint screen
        self.repaint()

    def setData(self, pols):
        # Set the polygons to be painted
        self.clearData()

        self.polygons = pols

        self.repaint()

    def greenPolygon(self, pol):
        # Set the polygon to be painted green
        self.greenPol = pol
        self.repaint()

    def paintRED(self, e: QPaintEvent):
        self.paint_red = True  # Set the flag to True when paintRED is called
        self.repaint()  # Trigger a repaint event

    def switchYellow(self):
        # Set the flag to False when switchYellow is called
        self.paint_red = False
