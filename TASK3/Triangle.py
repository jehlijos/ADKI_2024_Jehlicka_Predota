from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from QPoint3DF import *

class Triangle:
    def __init__(self, p1: QPoint3DF, p2: QPoint3DF, p3: QPoint3DF, slope: float, aspect: float):
        self.vertices = [p1, p2, p3]
        self.slope = slope
        self.aspect = aspect

    def getVertices(self):
        # Method to get the vertices of the triangle
        return QPolygonF([QPointF(p.x(), p.y()) for p in self.vertices])

    def getSlope(self):
        # Method to get the slope of the triangle
        return self.slope

    def getAspect(self):
        # Method to get the aspect of the triangle
        return self.aspect

    def getZValues(self):
        # Method to get the z-values of the vertices of the triangle
        return [p.getZ() for p in self.vertices]

    def getMinZ(self):
        # Method to get the minimum z-value of the triangle
        return min(self.getZValues())

    def getMaxZ(self):
        # Method to get the maximum z-value of the triangle
        return max(self.getZValues())

