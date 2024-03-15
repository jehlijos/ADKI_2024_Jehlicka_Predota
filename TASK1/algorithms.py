from PyQt6.QtCore import *
from PyQt6.QtGui import *
from math import *


class Algorithms:

    def __init__(self):
        pass

    def getPointPolPosition(self, q: QPointF, pol: QPolygonF):
        # Initialize variables
        j = 0
        n = len(pol)
        vert = 0

        # Iterate through polygon edges
        for i in range(n):
            # Determine edge indices
            if i != (n - 1):
                a = i
                b = i + 1
            else:
                a = i
                b = 0

            # Calculate point-edge distances
            x_red = pol[a].x() - q.x()
            y_red = pol[a].y() - q.y()

            xi1_red = pol[b % n].x() - q.x()
            yi1_red = pol[b % n].y() - q.y()

            # Check for intersection
            if ((yi1_red > 0) and (y_red <= 0)) or ((y_red > 0) and (yi1_red <= 0)):
                x_int = (xi1_red * y_red - x_red * yi1_red) / (yi1_red - y_red)
                if x_int > 0:
                    j = j + 1

            # Check if point is on an edge
            det = ((pol[b].x() - pol[a].x()) * (q.y() - pol[a].y())) - (
                        (pol[b].y() - pol[a].y()) * (q.x() - pol[a].x()))
            if det == 0:
                vert = vert + 1

        # Determine point position relative to polygon
        if vert == 1:
            return 1
        if vert > 1:
            return 2
        if j % 2 == 1:
            return 3
        return 0

    def windingNumber(self, q: QPointF, pol: QPolygonF):
        # Initialize variables
        w = 0
        n = len(pol)
        vert = 0

        # Iterate through polygon edges
        for i in range(n):
            a = i
            b = (i + 1) % n

            # Calculate distances between point and vertices
            x_qi_del = pol[a].x() - q.x()
            y_qi_del = pol[a].y() - q.y()

            l_qi = sqrt(x_qi_del * x_qi_del + y_qi_del * y_qi_del)

            x_qi1_del = pol[b].x() - q.x()
            y_qi1_del = pol[b].y() - q.y()

            l_qi1 = sqrt(x_qi1_del * x_qi1_del + y_qi1_del * y_qi1_del)

            x_ii1_del = pol[a].x() - pol[b].x()
            y_ii1_del = pol[a].y() - pol[b].y()

            l_ii1 = sqrt(x_ii1_del * x_ii1_del + y_ii1_del * y_ii1_del)

            cosine_value = (l_qi * l_qi + l_qi1 * l_qi1 - l_ii1 * l_ii1) / (2 * l_qi * l_qi1)

            # Check for special cases
            if l_qi == 0 or l_qi1 == 0 or cosine_value > 1 or cosine_value < -1:
                vert = 5
            else:
                w2 = acos(cosine_value)

            # Determine winding number
            det = ((pol[b].x() - pol[a].x()) * (q.y() - pol[a].y())) - (
                        (pol[b].y() - pol[a].y()) * (q.x() - pol[a].x()))

            if det > 0:
                w = w + w2
            elif det < 0:
                w = w - w2

            if det == 0:
                vert = vert + 1

        # Determine point position relative to polygon
        if vert > 1:
            return 1
        if vert == 1:
            return 2
        if abs(abs(w) - 2 * pi) < 0.01:
            return 3
        return 0
