import math
import bezier
import numpy as np
from BasicGeometry import BasicGeometry

class Vehicle:
    # initialise a vehicle, setting its x,y coord and heading
    def __init__(self,x,y,theta):
        self._x = x
        self._y = y
        self._theta = theta
      

    @property
    def x(self):
        return self._x
    @property
    def y(self):
        return self._y
    @property
    def theta(self):
        return self._theta

    def __eq__(self, other):
        if other == None:
            return False
        else:
            return (round(self.x,2) == round(other.x,2)) and (round(self.y,2) == round(other.y,2)) and (round(self.theta,0) == round(other.theta,0))

    def __str__(self):
        return "Vehicle at (%.5f,%.5f) heading [%.5f]" % (self.x,self.y,self.theta)

    def __hash__(self):
        t = (self._x,self._y,self._theta)
        return hash(t)

    def applyControl(self,radius,deltaTheta,direction):
        if direction == "F" or direction == "R":
            a = 0
            b = radius
        else:
            a = radius*(1-math.cos(math.radians(deltaTheta)))
            b = radius*math.sin(math.radians(deltaTheta))
        x2 = None
        y2 = None
        theta2 = None
        if direction == "FL" or direction == "F":
            x2 = self._x+math.cos(math.radians(self._theta)-math.pi/2)*(-1*a)-math.sin(math.radians(self._theta)-math.pi/2)*(b)
            y2 = self._y+math.sin(math.radians(self._theta)-math.pi/2)*(-1*a)+math.cos(math.radians(self._theta)-math.pi/2)*(b)
            theta2 = self._theta + deltaTheta
        elif direction == "FR":
            deltaTheta *= -1.0
            x2 = self._x + math.cos(math.radians(self._theta)-math.pi/2)*(a)-math.sin(math.radians(self._theta)-math.pi/2)*(b)
            y2 = self._y + math.sin(math.radians(self._theta)-math.pi/2)*(a)+math.cos(math.radians(self._theta)-math.pi/2)*(b)
            theta2 = self._theta + deltaTheta
        elif direction == "RL":
            deltaTheta *= -1.0
            x2 = self._x + math.cos(math.radians(self._theta)-math.pi/2)*(-1*a)-math.sin(math.radians(self._theta)-math.pi/2)*(-1*b)
            y2 = self._y + math.sin(math.radians(self._theta)-math.pi/2)*(-1*a)+math.cos(math.radians(self._theta)-math.pi/2)*(-1*b)
            theta2 = self._theta + deltaTheta
        elif direction == "RR" or direction == "R":
            x2 = self._x + math.cos(math.radians(self._theta)-math.pi/2)*(a) - math.sin(math.radians(self._theta)-math.pi/2)*(-1*b)
            y2 = self._y + math.sin(math.radians(self._theta)-math.pi/2)*(a) + math.cos(math.radians(self._theta)-math.pi/2)*(-1*b)
            theta2 = self._theta + deltaTheta
        else:
            raise Exception("Unknown direction passed to apply control function (%s)" % direction)



        return Vehicle(x2,y2,theta2%360) #apply the control (radius,deltaTheta) in the direction specified to generate a new Vehicle object

    def DistanceMetric(self,otherVehicle):
        p1 = (self._x,self._y)
        p2 = (otherVehicle.x,otherVehicle.y)
        euclidean_distance = BasicGeometry.ptDist(p1,p2)
        cosine_distance = 1 - math.cos(math.radians(otherVehicle.theta)-math.radians(self._theta))
        return euclidean_distance + cosine_distance

    def EuclideanDistance(self,otherVehicle):
        p1 = (self._x,self._y)
        p2 = (otherVehicle.x,otherVehicle.y)
        euclidean_distance = BasicGeometry.ptDist(p1,p2)
        return euclidean_distance

    def createBezierCurveControl(self,otherVehicle):
        intersectionPoint = BasicGeometry.findVectorLinesIntersectionPoint(self.x,self.y,self.theta,otherVehicle.x,otherVehicle.y,otherVehicle.theta)
        if intersectionPoint == None:
            return False
        if intersectionPoint[0] == math.inf and intersectionPoint[1] == None:
            #deal with straight line case
            pass
        elif intersectionPoint[0] == None and intersectionPoint[1] == math.inf:
            #deal with straight line case
            pass
        print(intersectionPoint)
        x_points = [self.x,intersectionPoint[0],otherVehicle.x]
        y_points = [self.y,intersectionPoint[1],otherVehicle.y]
        nodes = np.asfortranarray([x_points,y_points])
        curve = bezier.Curve(nodes,degree=2)
        return curve

