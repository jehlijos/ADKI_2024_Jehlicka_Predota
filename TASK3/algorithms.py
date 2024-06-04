from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from math import *
from numpy import *
from numpy.linalg import *  #scipy.linalg
from QPoint3DF import *
from Edge import *
from decimal import *
from Triangle import *


class Algorithms:

    def __init__(self):
        pass

    def getPointPolPosition(self, q: QPoint3DF, pol: QPolygonF):
            # Point and polygon position, ray crossing algorithm
            k = 0
            n = len(pol)

            # Process all vertices of the polygon
            for i in range(n):
                # Reduce coordinates
                x_ir = pol[i].x() - q.x()
                y_ir = pol[i].y() - q.y()

                x_i1r = pol[(i + 1) % n].x() - q.x()
                y_i1r = pol[(i + 1) % n].y() - q.y()

                # Appropriate segment intersection the ray
                if ((y_i1r > 0) and (y_ir <= 0)) or ((y_ir > 0) and (y_i1r <= 0)):

                    # Compute intersection coordinate
                    xm = (x_i1r * y_ir - x_ir * y_i1r) / (y_i1r - y_ir)

                    # Appropriate intersection, increment k
                    if xm > 0:
                        k = k + 1

            # Inside
            if k % 2 == 1:
                return 1

            # Outside
            return 0

    def getSlope(self, p1: QPoint3DF, p2: QPoint3DF, p3: QPoint3DF):
        # Compute triangle slope

        # Vectors
        ux = p1.x() - p2.x()
        uy = p1.y() - p2.y()
        uz = p1.getZ() - p2.getZ()

        vx = p3.x() - p2.x()
        vy = p3.y() - p2.y()
        vz = p3.getZ() - p2.getZ()

        # Normal vector
        nx = uy * vz - uz * vy
        ny = -ux * vz + uz * vx
        nz = ux * vy - uy * vx

        # Norm
        norm = sqrt(nx ** 2 + ny ** 2 + nz ** 2)

        # Slope
        return acos(fabs(nz) / norm)

    def get2VectorsAngle(self, p1: QPoint3DF, p2: QPoint3DF, p3: QPoint3DF, p4: QPoint3DF):
        # Angle between two vectors
        ux = p2.x() - p1.x()
        uy = p2.y() - p1.y()

        vx = p4.x() - p3.x()
        vy = p4.y() - p3.y()

        # ot product
        dot = ux * vx + uy * vy

        # Norms
        nu = (ux ** 2 + uy ** 2) ** 0.5
        nv = (vx ** 2 + vy ** 2) ** 0.5

        # Correct interval
        arg = dot / (nu * nv)
        arg = max(-1, min(1, arg))

        return acos(arg)

    def createCH(self, pol: QPolygonF):
        # reate Convex Hull using Jarvis Scan
        ch = QPolygonF()

        # Find pivot q (minimize y)
        q = min(pol, key=lambda k: k.y())

        # Find left-most point (minimize x)
        s = min(pol, key=lambda k: k.x())

        # Initial segment
        pj = q
        pj1 = QPoint3DF(s.x(), q.y())

        # Add to CH
        ch.append(pj)

        # Find all points of CH
        while True:
            # Maximum and its index
            omega_max = 0
            index_max = -1

            # Browse all points
            for i in range(len(pol)):

                if pj != pol[i]:

                    # Compute omega
                    omega = self.get2VectorsAngle(pj, pj1, pj, pol[i])

                    # Actualize maximum
                    if (omega > omega_max):
                        omega_max = omega
                        index_max = i

            # Add point to the convex hull
            ch.append(pol[index_max])

            # Reasign points
            pj1 = pj
            pj = pol[index_max]

            # Stopping condition
            if pj == q:
                break

        return ch

    def createMMB(self, pol: QPolygonF):
        # Create min max box and compute its area

        # Points with extreme coordinates
        p_xmin = min(pol, key=lambda k: k.x())
        p_xmax = max(pol, key=lambda k: k.x())
        p_ymin = min(pol, key=lambda k: k.y())
        p_ymax = max(pol, key=lambda k: k.y())

        # Create vertices
        v1 = QPoint3DF(p_xmin.x(), p_ymin.y())
        v2 = QPoint3DF(p_xmax.x(), p_ymin.y())
        v3 = QPoint3DF(p_xmax.x(), p_ymax.y())
        v4 = QPoint3DF(p_xmin.x(), p_ymax.y())

        # Create new polygon
        mmb = QPolygonF([v1, v2, v3, v4])

        # Area of MMB
        area = (v2.x() - v1.x()) * (v3.y() - v2.y())

        return mmb, area

    def LH(self, pol: QPolygonF):
        # Compute polygon area using LH formula
        area = 0
        n = len(pol)

        # Compute area
        for i in range(n):
            area = area + pol[i].x() * (pol[(i + 1) % n].y() - pol[(i - 1 + n) % n].y())

        return abs(area) / 2

    def rotatePolygon(self, pol: QPolygonF, sig: float):
        # Rotate polygon according to a given angle
        pol_rot = QPolygonF()

        # Process all polygon vertices
        for i in range(len(pol)):
            # Rotate point
            x_rot = pol[i].x() * cos(sig) - pol[i].y() * sin(sig)
            y_rot = pol[i].x() * sin(sig) + pol[i].y() * cos(sig)

            # Create QPoint
            vertex = QPoint3DF(x_rot, y_rot)

            # Add vertex to rotated polygon
            pol_rot.append(vertex)

        return pol_rot

    def createMBR(self, pol: QPolygonF):
        # Create minimum area enclosing rectangle

        # Create convex hull
        ch = self.createCH(pol)

        # Get min-max box, area and sigma
        mmb_min, area_min = self.createMMB(ch)
        sigma_min = 0

        # Process all segments of ch
        for i in range(len(ch) - 1):

            # Compute sigma
            dx = ch[i + 1].x() - ch[i].x()
            dy = ch[i + 1].y() - ch[i].y()
            sigma = atan2(dy, dx)

            # Rotate convex hull by sigma
            ch_rot = self.rotatePolygon(ch, -sigma)

            # Find min-max box over rotated convex hull
            mmb, area = self.createMMB(ch_rot)

            # Actualize minimum area
            if area < area_min:
                area_min = area
                mmb_min = mmb
                sigma_min = sigma

        # Rotate min-max box
        er = self.rotatePolygon(mmb_min, sigma_min)

        # Resize rectangle
        er_r = self.resizeRectangle(er, pol)

        return er_r

    def resizeRectangle(self, er: QPolygonF, pol: QPolygonF):
        # Building area
        Ab = abs(self.LH(pol))

        # Enclosing rectangle area
        A = abs(self.LH(er))

        # Fraction of Ab and A
        k = Ab / A

        # Center of mass
        x_t = (er[0].x() + er[1].x() + er[2].x() + er[3].x()) / 4
        y_t = (er[0].y() + er[1].y() + er[2].y() + er[3].y()) / 4

        # Vectors
        u1_x = er[0].x() - x_t
        u2_x = er[1].x() - x_t
        u3_x = er[2].x() - x_t
        u4_x = er[3].x() - x_t
        u1_y = er[0].y() - y_t
        u2_y = er[1].y() - y_t
        u3_y = er[2].y() - y_t
        u4_y = er[3].y() - y_t

        # Coordinates of new vertices
        v1_x = x_t + sqrt(k) * u1_x
        v1_y = y_t + sqrt(k) * u1_y

        v2_x = x_t + sqrt(k) * u2_x
        v2_y = y_t + sqrt(k) * u2_y

        v3_x = x_t + sqrt(k) * u3_x
        v3_y = y_t + sqrt(k) * u3_y

        v4_x = x_t + sqrt(k) * u4_x
        v4_y = y_t + sqrt(k) * u4_y

        # Create new vertices
        v1 = QPoint3DF(v1_x, v1_y)
        v2 = QPoint3DF(v2_x, v2_y)
        v3 = QPoint3DF(v3_x, v3_y)
        v4 = QPoint3DF(v4_x, v4_y)

        # Create rectangle
        er_r = QPolygonF([v1, v2, v3, v4])

        return er_r

    def createERPCA(self, pol: QPolygonF):
        # Create enclosing rectangle using PCA
        x = []
        y = []

        # Add x,y coordinates to the list
        for p in pol:
            x.append(p.x())
            y.append(p.y())

        # Invert to matrix
        A = array([x, y])

        # Covariance matrix
        C = cov(A)

        # Angular value decomposition
        [U, S, V] = svd(C)

        # Compute sigma
        sigma = atan2(V[0][1], V[0][0])

        # Rotate polygon
        pol_rot = self.rotatePolygon(pol, -sigma)

        # Find min-max box over rotated building
        mmb, area = self.createMMB(pol_rot)

        # Rotate min-max box
        er = self.rotatePolygon(mmb, sigma)

        # Resize rectangle
        er_r = self.resizeRectangle(er, pol)

        return er_r

    def getNearestPoint(self, q: QPoint3DF, points: list[QPoint3DF]):
        # Return point nearest to q

        d_min = inf
        i_min = -1

        # Process all points of the cloud
        for i in range(len(points)):

            # q different from points[i]
            if q != points[i]:
                # Compute distance
                dx = q.x() - points[i].x()
                dy = q.y() - points[i].y()

                d = sqrt(dx ** 2 + dy ** 2)

                # Update minimum
                if d < d_min:
                    d_min = d
                    i_min = i

        return d_min, i_min

    def getPointAndLinePosition(self, p: QPoint3DF, p1: QPoint3DF, p2: QPoint3DF):
        # Analyze point and line position

        # Compute vectors u, v
        ux = p2.x() - p1.x()
        uy = p2.y() - p1.y()

        vx = p.x() - p1.x()
        vy = p.y() - p1.y()

        # Compute test
        t = ux * vy - uy * vx

        # Point if the left half plane
        if t > 0:
            return 1

        # Point if the right half plane
        if t < 0:
            return 0

        # Point on the line
        return -1

    def getDelaunayPoint(self, start: QPoint3DF, end: QPoint3DF, points: list[QPoint3DF]):
        # Return Delaunay point

        omega_max = 0
        i_max = -1

        # Process all points of the cloud
        for i in range(len(points)):

            # Start and end different from points[i]
            if start != points[i] and end != points[i]:

                # Point in left half-plaine
                if self.getPointAndLinePosition(points[i], start, end) == 1:

                    # Compute angle
                    omega = self.get2VectorsAngle(points[i], start, points[i], end)

                    # Update maximum
                    if omega > omega_max:
                        omega_max = omega
                        i_max = i

        return omega_max, i_max

    def createDT(self, points: list[QPoint3DF]):

        if len(points) < 3:
            return []

        # Create Delaunay triangulation
        dt = []
        ael = []

        # Find initial point
        p1 = min(points, key=lambda k: k.x())

        # Find nearest point
        d_min, idx = self.getNearestPoint(p1, points)
        p2 = points[idx]

        # Create edge and opposite edge
        e = Edge(p1, p2)
        e_op = Edge(p2, p1)

        # Add to ael
        ael.append(e)
        ael.append(e_op)

        # Repeat until ael is empty
        while ael:
            # Take the 1st edge
            e1 = ael.pop()

            # Switch orientation
            e1_op = e1.changeOrientation()

            # Find optimal Delaunay point
            omega_max, idx = self.getDelaunayPoint(e1_op.getStart(), e1_op.getEnd(), points)

            # Is there any Delaunay point
            if idx >= 0:
                # Create remaining edges
                e2 = Edge(e1_op.getEnd(), points[idx])
                e3 = Edge(points[idx], e1_op.getStart())

                # Add triangle to dt
                dt.append(e1_op)
                dt.append(e2)
                dt.append(e3)

                # Update
                self.updateAEL(e2, ael)
                self.updateAEL(e3, ael)

        return dt

    def updateAEL(self, e: Edge, ael: list[Edge]):
        # Update list of valid Delaunay edges

        # Change orientation
        e_op = e.changeOrientation()

        # Is edge in AEL?
        # Yes, remove edge
        if e_op in ael:
            ael.remove(e_op)

        # No, add to the list
        else:
            ael.append(e)

    def getContourPoint(self, p1: QPoint3DF, p2: QPoint3DF, z: float):
        # Compute intersection point

        xb = (p2.x() - p1.x()) / (p2.getZ() - p1.getZ()) * (z - p1.getZ()) + p1.x()
        yb = (p2.y() - p1.y()) / (p2.getZ() - p1.getZ()) * (z - p1.getZ()) + p1.y()

        return QPoint3DF(xb, yb, z)

    def CreateCountourLines(self, dt: list[Edge], zmin: float, zmax: float, dz: float):
        # Create contour lines inside interval zmin, zmax and with step dz
        contours = []

        # Process all triangles
        for i in range(0, len(dt), 3):
            # Get triangle verticies
            p1 = dt[i].getStart()
            p2 = dt[i].getEnd()
            p3 = dt[i + 1].getEnd()

            # Z of points
            z1 = p1.getZ()
            z2 = p2.getZ()
            z3 = p3.getZ()

            # Test horizontal plane and triangle intersections
            for z in range(zmin, zmax, dz):
                # mpute height differences
                dz1 = z - z1
                dz2 = z - z2
                dz3 = z - z3

                # Triangle is coplanar
                if dz1 == 0 and dz2 == 0 and dz3 == 0:
                    continue

                # Edge (1, 2) in plane
                elif dz1 == 0 and dz2 == 0:
                    contours.append(dt[i])

                # Edge (2, 3) in plane
                elif dz2 == 0 and dz3 == 0:
                    contours.append(dt[i + 1])

                # Edge (3, 1) in plane
                elif dz3 == 0 and dz1 == 0:
                    contours.append(dt[i + 2])

                # Edges (1,2) & (2,3) intersected by plane
                elif dz1 * dz2 <= 0 and dz2 * dz3 < 0 or dz1 * dz2 < 0 and dz2 * dz3 <= 0:
                    # Contour intersections
                    a = self.getContourPoint(p1, p2, z)
                    b = self.getContourPoint(p2, p3, z)

                    # Create new edge/line
                    e = Edge(a, b)

                    # Add edge to the list
                    contours.append(e)

                # Edges (2,3) & (3,1) intersected by plane
                elif dz2 * dz3 <= 0 and dz3 * dz1 < 0 or dz2 * dz3 < 0 and dz3 * dz1 <= 0:
                    # Contour intersections
                    a = self.getContourPoint(p2, p3, z)
                    b = self.getContourPoint(p3, p1, z)

                    # Create new edge/line
                    e = Edge(a, b)

                    # Add edge to the list
                    contours.append(e)

                # Edges (3,1) & (1,2) intersected by plane
                elif dz3 * dz1 <= 0 and dz1 * dz2 < 0 or dz3 * dz1 < 0 and dz1 * dz2 <= 0:
                    # Contour intersections
                    a = self.getContourPoint(p3, p1, z)
                    b = self.getContourPoint(p1, p2, z)

                    # Create new edge/line
                    e = Edge(a, b)

                    # Add edge to the list
                    contours.append(e)

        return contours

    def getAspect(self, p1: QPoint3DF, p2: QPoint3DF, p3: QPoint3DF):
        # Compute triangle slope

        # Vectors
        ux = p1.x() - p2.x()
        uy = p1.y() - p2.y()
        uz = p1.getZ() - p2.getZ()

        vx = p3.x() - p2.x()
        vy = p3.y() - p2.y()
        vz = p3.getZ() - p2.getZ()

        # Normal vector
        nx = uy * vz - uz * vy
        ny = -ux * vz + uz * vx

        # Aspect
        return atan2(nx, ny)

    def analyzeDTMSlopeAndAspect(self, dt):
        # Analyze DTM slope and aspect
        triangles = []

        # Process all triangles
        for i in range(0, len(dt), 3):
            # Get triangle verticies
            p1 = dt[i].getStart()
            p2 = dt[i].getEnd()
            p3 = dt[i + 1].getEnd()

            # Compute slope
            slope = self.getSlope(p1, p2, p3)

            # Compute aspect
            aspect = self.getAspect(p1, p2, p3)

            # Create triangle
            triangle = Triangle(p1, p2, p3, slope, aspect)

            # Add triangle to list
            triangles.append(triangle)

        return triangles
