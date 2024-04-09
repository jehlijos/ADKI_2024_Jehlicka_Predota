from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from shapely.geometry import shape
from fiona import *
from statistics import *


class IO:

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dia = QFileDialog()
        self.dia.setNameFilter("Shapefile (*.shp)")

    def loadGeometries(self, fileName):
        # Method to load geometries from a Shapefile
        geometries = []
        # Opening the Shapefile
        with open(fileName) as shapefile:
            # Iterating through each record in the Shapefile
            for record in shapefile:
                geom = shape(record['geometry'])
                geometries.append(geom)
        # Returning list of geometries
        return geometries

    def createPolygons(self, geometries):
        # Method to create PyQt polygons from Shapely geometries
        polygons = []
        # Iterating through each geometry
        for pol in geometries:
            qpolygon = QPolygonF()
            # Iterating through each point in the geometry
            for point in pol.exterior.coords:
                # Adding each point to the Qt polygon
                qpolygon.append(QPointF(point[0], point[1] * (-1)))
            polygons.append(qpolygon)
        return polygons

    def scaleAndTranslatePolygons(self, polygons, s, shift_x, shift_y):
        # Method to scale and translate polygons

        # Creating scaled and translated polygon iterating through each polygon
        Data = [QPolygonF([QPointF(point.x() * s - shift_x, point.y() * s - shift_y)
                           for point in pol]) for pol in polygons]
        return Data

    def processPolygonPoints(self, pol):
        # Method to process points of a polygon

        # Initializing extreme values
        x_min = float('inf')
        y_min = float('inf')
        x_max = float('-inf')
        y_max = float('-inf')

        x_crds = []
        y_crds = []

        # Iterating through each point in the polygon
        for point in pol:
            x = point.x()
            y = point.y()

            x_crds.append(x)
            y_crds.append(y)
            # Updating extreme coordinate if necessary
            x_min = min(x_min, x)
            y_min = min(y_min, y)
            x_max = max(x_max, x)
            y_max = max(y_max, y)

        return x_min, y_min, x_max, y_max, x_crds, y_crds

    def loadData(self, w, h):
        # Method to load Shapefile after selecting it
        if self.dia.exec():

            # Initializing extreme values
            x_pol_min = float('inf')
            y_pol_min = float('inf')
            x_pols_max = float('-inf')
            y_pols_max = float('-inf')

            # Loading geometries from selected Shapefile
            geometries = self.loadGeometries(self.dia.selectedFiles()[0])
            # Creating polygons from loaded geometries
            polygons = self.createPolygons(geometries)

            all_x = []
            all_y = []

            # Iterating through each polygon
            for pol in polygons:
                min_x, min_y, max_x, max_y, points_x, points_y = self.processPolygonPoints(pol)

                # Appending coordinates to the list
                all_x += points_x
                all_y += points_y

                # Updating extreme coordinates of polygons if necessary
                x_pol_min = min(x_pol_min, min_x)
                y_pol_min = min(y_pol_min, min_y)
                x_pols_max = max(x_pols_max, max_x)
                y_pols_max = max(y_pols_max, max_y)

            # Calculating height and weight of bounding box
            H = y_pols_max - y_pol_min
            W = x_pols_max - x_pol_min

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

            Data = []
            # Iterating through each polygon
            for pol in polygons:
                polygon = QPolygonF()
                points = {}
                # Iterating through each point in the polygon
                for point in pol:
                    # Scaling and shifting coordinates
                    x = point.x() * scale - shift_x
                    y = point.y() * scale - shift_y

                    # Checking if point is unique
                    if x not in points:
                        points[x] = y

                        # Storing Y-coordinate corresponding to X-coordinate
                        point2 = QPointF(x, y)

                        # Creating a QPointF object with scaled and shifted coordinates
                        polygon.append(point2)

                Data.append(polygon)

            return Data
