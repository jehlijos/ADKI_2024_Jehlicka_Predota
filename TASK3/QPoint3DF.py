from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from shapely.geometry import shape, Point
from fiona import open as fiona_open
from statistics import mean


class QPoint3DF(QPointF):
    def __init__(self, x: float, y: float, z: float):
        super().__init__(x, y)
        self.z = z

    def getZ(self):
        # Method to get the z-coordinate of the point
        return self.z

