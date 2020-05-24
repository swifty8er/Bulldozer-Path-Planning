import bezier
import numpy as np
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
            t += 0.001
        return curve



