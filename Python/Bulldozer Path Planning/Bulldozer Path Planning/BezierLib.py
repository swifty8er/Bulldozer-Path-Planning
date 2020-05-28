import bezier
import numpy as np
import math
import queue
import random
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry

MIN_RADIUS = 0.4


class BezierLib():
    @staticmethod
    def createBezierCurveBetweenTwoVehicle(v1,v2):
        dist = v1.EuclideanDistance(v2)
        r = dist/4.0
        x_points = [v1.x,v1.x+r*math.cos(math.radians(v1.theta)),v2.x-r*math.cos(math.radians(v2.theta)),v2.x]
        y_points = [v1.y,v1.y+r*math.sin(math.radians(v1.theta)),v2.y-r*math.sin(math.radians(v2.theta)),v2.y]
        nodes = np.asfortranarray([x_points,y_points])
        curve = bezier.Curve(nodes,degree=3)
        t = 0.0
        while t<=1.0:
            kappa = BasicGeometry.evaluateKappa(curve,t)
            if round(kappa,2) == 0:
                roC = math.inf
            else:
                roC = 1.0/kappa
            if roC < MIN_RADIUS:
                return False
            t += 0.005

        return curve


    @staticmethod
    def testCurve(curve,map,curr_disk_positions):
        s = 0.0
        edges = map.getMapEdgesAndObstacles()
        while s <= 1.0:
            kappa = BasicGeometry.evaluateKappa(curve,s)
            if round(kappa,2) == 0:
                radiusOfCurvature = math.inf
            else:
                radiusOfCurvature = 1.0/kappa
            if radiusOfCurvature < MIN_RADIUS:
                return False
            point_list = curve.evaluate(s)
            point = [i[0] for i in point_list]
            for edge in edges:
                if map.disk_radius - BasicGeometry.point2LineDist(edge,point) > np.finfo(np.float32).eps:
                    return False
            for disk_pos in curr_disk_positions:
                if 1.95 * map.disk_radius - BasicGeometry.ptDist(disk_pos,point) > np.finfo(np.float32).eps:
                    return False
            s+= 0.005
        return True


    @staticmethod
    def getBestBezierCurveConnectionBetweenTwoPoses(pose1,pose2,map,curr_disk_positions,degree,iterations,max_num_candidates):
        curves = []
        num_control_points = degree-3
        dist = pose1.EuclideanDistance(pose2)
        r = dist/5.0
        x_points_start = [pose1.x,pose1.x+r*math.cos(math.radians(pose1.theta))]
        x_points_end = [pose2.x-r*math.cos(math.radians(pose2.theta)),pose2.x]
        y_points_start = [pose1.y,pose1.y+r*math.sin(math.radians(pose1.theta))]
        y_points_end = [pose2.y-r*math.sin(math.radians(pose2.theta)),pose2.y]
        x = 0
        while x < iterations and len(curves) < max_num_candidates:
            x_points_middle = []
            y_points_middle = []
            for i in range(num_control_points):
                x_points_middle.append(random.uniform(map.min_x,map.max_x))
                y_points_middle.append(random.uniform(map.min_y,map.max_y))
            x_points = x_points_start + x_points_middle + x_points_end
            y_points = y_points_start + y_points_middle + y_points_end
            nodes = np.asfortranarray([x_points,y_points])
            curve = bezier.Curve(nodes,degree=degree)
            if BezierLib.testCurve(curve,map,curr_disk_positions):
                curves.append(curve)
            x+=1

        if len(curves) == 0:
            return False
        else:
            bestCurve = None
            shortestLength = math.inf
            for c in curves:
                if c.length < shortestLength:
                    bestCurve = c

            return bestCurve