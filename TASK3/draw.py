from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from Edge import *
from QPoint3DF import *
from random import *
from Triangle import *
from math import *

class Draw(QWidget):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.points = []
        self.dt = []
        self.contours = []
        self.triangles = []

        self.draw_dt = True
        self.draw_contours = True
        self.draw_slope = True
        self.draw_aspect = True
        self.draw_hypso = True

    def mousePressEvent(self, e: QMouseEvent):
        # Block using in Pane class in TD.py

        # Get cursor position
        x = e.position().x()
        y = e.position().y()

        # Get zmin and zmax values from the configuration file
        with open("settings.conf", "r") as file:
            # Reading the configuration file
            lines = file.readlines()
            # Extracting the values of zmin, zmax
            zmin = int(round(float(lines[0].strip())))
            zmax = int(round(float(lines[1].strip())))

        z = random() * (zmax - zmin) + zmin

        # Create new point
        p = QPoint3DF(x, y, z)

        # Add point to point cloud
        self.points.append(p)

        # Repaint screen
        self.repaint()

    def paintEvent(self, e: QPaintEvent):
        # Draw situation
        qp = QPainter(self)
        qp.begin(self)

        # Draw hypsometric tints
        if self.draw_hypso and self.triangles:

            # Get zmin and zmax values from vertices for all triangles
            zmin = min(t.getMinZ() for t in self.triangles)
            zmax = max(t.getMaxZ() for t in self.triangles)

            # Paint lowest triangle green and highest brown
            for t in self.triangles:
                # Get the Z values of the triangle vertices
                z1, z2, z3 = t.getZValues()
                meanZ = (z1 + z2 + z3) / 3

                # Normalize the Z value from zmin to zmax to a range of 0 to 1
                normalizedZ = (meanZ - zmin) / (zmax - zmin)

                # Calculate RGB values based on the normalized Z value
                # Using colors scheme from dark green to red
                #
                if normalizedZ < 1 / 3:
                    r = 0
                    g = int(255 * (normalizedZ * 3))
                    b = 0
                #
                elif normalizedZ < 2 / 3:
                    r = int(255 * (normalizedZ - 1 / 3) * 3)
                    g = 255
                    b = 0
                else:
                    r = 255
                    g = int(255 * (1 - (normalizedZ - 2 / 3) * 3))
                    b = 0

                # Create a QColor from the calculated RGB values
                col = QColor(r, g, b)

                # Create a QPolygonF from the triangle vertices
                polygon = t.getVertices()

                # Set the brush to the calculated color
                qp.setBrush(col)

                # Draw the polygon
                qp.drawPolygon(polygon)

        if self.draw_slope:
            # Draw triangles: slope
            for t in self.triangles:
                vertices = t.getVertices()
                slope = t.getSlope()
                RGB = int(255 - slope * 2 * 255 / pi)
                col = QColor(RGB, RGB, RGB)
                qp.setBrush(col)
                qp.drawPolygon(vertices)

        if self.draw_aspect:
            # Draw triangles: aspect
            for t in self.triangles:
                vertices = t.getVertices()
                aspect = t.getAspect()

                # Normalize the aspect value from -pi to +pi to a range of 0 to 1
                normalized_aspect = (aspect + pi) / (2 * pi)

                # Calculate RGB values based on the normalized aspect value
                if 0 <= normalized_aspect < 1 / 4:  # Blue to Green
                    r = 0
                    g = int(255 * (normalized_aspect * 4))
                    b = int(255 * (1 - normalized_aspect * 4))
                elif 1 / 4 <= normalized_aspect < 1 / 2:  # Green to Yellow
                    r = int(255 * (normalized_aspect - 1 / 4) * 4)
                    g = 255
                    b = 0
                elif 1 / 2 <= normalized_aspect < 3 / 4:  # Yellow to Red
                    r = 255
                    g = int(255 * (1 - (normalized_aspect - 1 / 2) * 4))
                    b = 0
                else:  # Red to Blue
                    r = int(255 * (1 - (normalized_aspect - 3 / 4) * 4))
                    g = 0
                    b = int(255 * ((normalized_aspect - 3 / 4) * 4))

                # Create a QColor from the calculated RGB values
                col = QColor(r, g, b)

                # Set the brush to the calculated color
                qp.setBrush(col)

                # Draw the polygon with the specified vertices
                qp.drawPolygon(vertices)

        # Draw Delaunay triangulation
        if self.draw_dt:
            qp.setPen(QPen(Qt.GlobalColor.green))
            for e in self.dt:
                qp.drawLine(int(e.getStart().x()), int(e.getStart().y()), int(e.getEnd().x()), int(e.getEnd().y()))

        # Draw contour lines
        qp.setPen(QPen(Qt.GlobalColor.red, 2))

        if self.draw_contours:
            # Draw contour lines
            # Get unique Z values and sort them
            unique_z_values = sorted(set(e.getEnd().getZ() for e in self.contours))
            # Create a dictionary to map Z values to indices
            z_value_to_index = {z: idx for idx, z in enumerate(unique_z_values)}
            # Counter to keep track of the number of contour segments drawn
            text_counter = 0

            for e in self.contours:
                z_value = e.getEnd().getZ()
                if (z_value_to_index[z_value] + 1) % 5 == 0:
                    qp.setPen(QPen(Qt.GlobalColor.red, 4))  # Twice as wide for every 5th unique Z value
                else:
                    qp.setPen(QPen(Qt.GlobalColor.red, 2))
                qp.drawLine(int(e.getStart().x()), int(e.getStart().y()), int(e.getEnd().x()), int(e.getEnd().y()))

                # Set font size and draw Z value only for every 5th contour segment
                text_counter += 1
                if text_counter % 5 == 0:
                    font = qp.font()
                    font.setPointSize(12)
                    font.setBold(True)
                    qp.setFont(font)
                    qp.drawText(int(e.getEnd().x() + 5), int(e.getEnd().y() + 5), str(z_value))

        # Draw points as crosses
        qp.setPen(QPen(Qt.GlobalColor.blue, 2))
        length = 5
        # if p is NoneType, pass
        if self.points:
            for p in self.points:
                qp.drawLine(int(p.x()), int(p.y() - length), int(p.x()), int(p.y() + length))
                qp.drawLine(int(p.x() - length), int(p.y()), int(p.x() + length), int(p.y()))
        else:
            # Make p a empty list and not NoneType
            self.points = []

        qp.end()

    def getPoints(self):
        # Return points
        return self.points

    def clearAll(self):
        # Clear points
        self.points.clear()

        # Clear triangles
        self.dt.clear()

        # Clear DT
        self.dt.clear()

        # Clear contour lines
        self.contours.clear()

        # Clear triangles
        self.triangles.clear()

        # Repaint screen
        self.repaint()

    def clearResults(self):
        # Clear triangles
        self.dt.clear()

        # Clear contour lines
        self.contours.clear()

        # Clear triangles
        self.triangles.clear()

        # Repaint screen
        self.repaint()

    def setDT(self, dt: list[Edge]):
        # Set DT
        self.dt = dt

    def getDT(self):
        # Get DT
        return self.dt

    def setContours(self, contours: list[Edge]):
        # Set contour lines
        self.contours = contours

    def setTriangles(self, triangles: list[Triangle]):
        # Set triangles
        self.triangles = triangles

    def setDrawDT(self, draw_dt):
        # Set draw DT
        self.draw_dt = draw_dt

    def SetPoints(self, points):
        # Set points
        self.points = points

    def setDrawContours(self, draw_contours):
        # Set draw contour lines
        self.draw_contours = draw_contours

    def setDrawSlope(self, draw_slope):
        # Set draw slope
        self.draw_slope = draw_slope

    def setDrawAspect(self, draw_aspect):
        # Set draw aspect
        self.draw_aspect = draw_aspect

    def getContours(self):
        # Get contour lines
        return self.contours
