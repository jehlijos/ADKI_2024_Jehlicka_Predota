from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from shapely.geometry import shape, Point
from fiona import open as fiona_open
from statistics import mean
from QPoint3DF import *



class IO:

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dia = QFileDialog()
        self.dia.setNameFilter("Shapefile (*.shp)")

    def loadGeometries(self, fileName):
        # Method to load 3D point geometries from a Shapefile
        points = []
        # Opening the Shapefile
        with fiona_open(fileName) as shapefile:
            # Iterating through each record in the Shapefile
            for record in shapefile:
                geom = shape(record['geometry'])
                if isinstance(geom, Point):
                    x, y, z = geom.x, geom.y, geom.z if geom.has_z else 0
                    points.append(QPoint3DF(x, y, z))
        # Returning list of 3D points
        return points

    def createPoints(self, points):
        # Method to create PyQt 3D points from Shapely points
        qpoints = []
        # Iterating through each point
        for pt in points:
            qpoint = QPoint3DF(pt.x(), pt.y() * (-1), pt.getZ())
            qpoints.append(qpoint)
        return qpoints

    def scaleAndTranslatePoints(self, points, s, shift_x, shift_y):
        # Method to scale and translate points
        scaled_translated_points = [QPoint3DF(point.x() * s - shift_x, point.y() * s - shift_y, point.getZ())
                                    for point in points]
        return scaled_translated_points

    def processPointCoordinates(self, points):
        # Method to process coordinates of points
        x_crds = [point.x() for point in points]
        y_crds = [point.y() for point in points]
        z_crds = [point.getZ() for point in points]
        return x_crds, y_crds, z_crds

    def loadData(self, w, h):
        # Method to load Shapefile after selecting it
        if self.dia.exec():
            selected_files = self.dia.selectedFiles()
            if not selected_files:
                # For user closing the dialog without selecting any file
                exit()

            # Initializing extreme values
            x_min = float('inf')
            y_min = float('inf')
            x_max = float('-inf')
            y_max = float('-inf')

            # Loading point geometries from selected Shapefile
            points = self.loadGeometries(selected_files[0])
            # Creating QPoint3DF from loaded geometries
            qpoints = self.createPoints(points)

            all_x, all_y, all_z = self.processPointCoordinates(qpoints)

            # Updating extreme coordinates of points if necessary
            x_min = min(all_x)
            y_min = min(all_y)
            x_max = max(all_x)
            y_max = max(all_y)

            # Calculating height and width of bounding box
            H = y_max - y_min
            W = x_max - x_min

            # Calculating height ratio and width ratio
            ratio_h = h / H
            ratio_w = w / W

            mean_x = mean(all_x)
            mean_y = mean(all_y)

            # Calculating scaling factor
            scale = min(ratio_w, ratio_h) * 0.9

            center_X = mean_x * scale
            center_Y = mean_y * scale

            # Calculating center of the window
            center_x = w / 2
            center_y = h / 2

            # Calculating shift in X-direction and Y-direction
            shift_x = center_X - center_x
            shift_y = center_Y - center_y

            # Scaling and translating points
            Data = self.scaleAndTranslatePoints(qpoints, scale, shift_x, shift_y)

            return Data
        else:
            pass
