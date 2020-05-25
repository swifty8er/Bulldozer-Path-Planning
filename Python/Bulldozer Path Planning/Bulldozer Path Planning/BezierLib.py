import bezier
import numpy as np
import math
import random
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry

MIN_RADIUS = 0.4


class BezierLib():
    @staticmethod
    def createBezierCurveBetweenTwoVehicle(v1,v2):
        x_points = [v1.x,v1.x+math.cos(math.radians(v1.theta)),v2.x-math.cos(math.radians(v2.theta)),v2.x]
        y_points = [v1.y,v1.y+math.sin(math.radians(v1.theta)),v2.y-math.sin(math.radians(v2.theta)),v2.y]
        nodes = np.asfortranarray([x_points,y_points])
        curve = bezier.Curve(nodes,degree=3)
        t = 0.0
        while t<=1.0:
            kappa = BasicGeometry.evaluateKappa(curve,t)
            roC = 1.0/kappa
            if roC < MIN_RADIUS:
                return False
            t += 0.0025

        return curve

    @staticmethod
    def testRadiusOfCurvature(curve):
        s = 0.0
        while s <= 1.0:
            kappa = BasicGeometry.evaluateKappa(curve,s)
            radiusOfCurvature = 1.0/kappa
            if radiusOfCurvature < MIN_RADIUS:
                return False
            s += 0.001
        return True


    @staticmethod
    def testCollision(curve,map,disk_positions):
        edges = map.getMapEdgesAndObstacles()
        s = 0.0
        while s<=1.0:
            point_list = curve.evaluate(s)
            point = [i[0] for i in point_list]
            for edge in edges:
                if map.disk_radius - BasicGeometry.point2LineDist(edge,point) > np.finfo(np.float32).eps:
                    return False
            for disk_pos in disk_positions:
                if 2 * map.disk_radius - BasicGeometry.ptDist(disk_pos,point) > np.finfo(np.float32).eps:
                    return False

            s+= 0.005
        return True

    @staticmethod
    def getBestBezierCurveConnectionBetweenTwoPoses(pose1,pose2,map,curr_disk_positions,degree,iterations,max_num_candidates):
        curves = []
        num_control_points = degree-3
        x_points_start = [pose1.x,pose1.x+math.cos(math.radians(pose1.theta))]
        x_points_end = [pose2.x-math.cos(math.radians(pose2.theta)),pose2.x]
        y_points_start = [pose1.y,pose1.y+math.sin(math.radians(pose1.theta))]
        y_points_end = [pose2.y-math.sin(math.radians(pose2.theta)),pose2.y]
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
            if BezierLib.testRadiusOfCurvature(curve) and BezierLib.testCollision(curve,map,curr_disk_positions):
                print("Found valid curve")
                curves.append((curve.length,curve))
            x+=1

        if len(curves) == 0:
            return False
        else:
            curves.sort()
            return curves[0][0]



