import bezier
import numpy as np
import math
import queue
import random
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry

MIN_RADIUS = 0.4
MAX_ANGLE_DIFFERENCE = 5.0

class BezierLib():
    @staticmethod
    def getInverseCurve(curve):
        nodes = curve.nodes
        i = 0
        x_points = []
        y_points = []
        for n in nodes:
            for num in n:
                if i == 0:
                    x_points.append(num)
                else:
                    y_points.append(num)
            i+=1
        x_points.reverse()
        y_points.reverse()
        inv_nodes = np.asfortranarray([x_points,y_points])
        inv_curve = bezier.Curve(inv_nodes,curve.degree)
        return inv_curve

    @staticmethod
    def createBezierCurveBetweenTwoVehiclesIntersectionMethod(v1,v2):
        intersectionPoint = BasicGeometry.findVectorLinesIntersectionPoint(v1.x,v1.y,v1.theta,v2.x,v2.y,v2.theta)
        if intersectionPoint == None:
            return False
        if intersectionPoint[0] == math.inf and intersectionPoint[1] == None:
            dist = abs(v1.x-v2.x)
            if self.x<otherVehicle.x:
                return (dist,0,"F")
            else:
                return (dist,0,"R")
        elif intersectionPoint[0] == None and intersectionPoint[1] == math.inf:
            dist = abs(v1.y-v2.y)
            if self.y<otherVehicle.y:
                return (dist,0,"F")
            else:
                return (dist,0,"R")
        x_points = [v1.x,intersectionPoint[0],v2.x]
        y_points = [v1.y,intersectionPoint[1],v2.y]
        nodes = np.asfortranarray([x_points,y_points])
        curve = bezier.Curve(nodes,degree=2)

        s = 0.0
        dist = math.inf
        t = None
        while s <= 1.0:
            point_array = curve.evaluate(s).tolist()
            point = [i[0] for i in point_array]
            d = BasicGeometry.ptDist(intersectionPoint,point)
            if d < dist:
                dist = d
                t = s
            s += 0.1

        if t == None:
            raise Exception("No t value found")
        interval = 0.2
        # Binary search to find the closest point on the bezier curve to the intersection point
        # This is where the radius of curvature will be smallest
        while interval > 0.01:
            p1_array = curve.evaluate(t+interval/2.0).tolist()
            p1 = [i[0] for i in p1_array]
            p2_array = curve.evaluate(t-interval/2.0).tolist()
            p2 = [i[0] for i in p2_array]
            point_array = curve.evaluate(t).tolist()
            point = [i[0] for i in point_array]
            if BasicGeometry.ptDist(p1,intersectionPoint) < BasicGeometry.ptDist(point,intersectionPoint):
                t = t+interval/2.0
            elif BasicGeometry.ptDist(p2,intersectionPoint) < BasicGeometry.ptDist(point,intersectionPoint):
                t = t-interval/2.0
            else:
                interval = interval/2.0
        kappa = BasicGeometry.evaluateKappa(curve,t)
        if round(kappa,2) == 0:
            radiusOfCurvature = math.inf
        else:
            radiusOfCurvature = 1.0/kappa
        if radiusOfCurvature<MIN_RADIUS:
            return False
        tangentStart = BasicGeometry.getTangentAngleOfBezierCurveAtPoint(curve,0.0)
        tangentEnd = BasicGeometry.getTangentAngleOfBezierCurveAtPoint(curve,1.0)
        if (abs(tangentStart-v1.theta) > MAX_ANGLE_DIFFERENCE) or (abs(tangentEnd-v2.theta) > MAX_ANGLE_DIFFERENCE):
            return False
        return curve

    @staticmethod
    def createBezierCurveBetweenTwoVehicle(v1,v2,map,curr_disk_positions):
        distance = v1.EuclideanDistance(v2)
        r = distance / 4.0
        x_points = [v1.x,v1.x+r*math.cos(math.radians(v1.theta)),v2.x-r*math.cos(math.radians(v2.theta)),v2.x]
        y_points = [v1.y,v1.y+r*math.sin(math.radians(v1.theta)),v2.y-r*math.sin(math.radians(v2.theta)),v2.y]
        nodes = np.asfortranarray([x_points,y_points])
        curve = bezier.Curve(nodes,degree=3)
        if BezierLib.testCurve(curve,map,curr_disk_positions):
            return curve
        else:
            return False

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