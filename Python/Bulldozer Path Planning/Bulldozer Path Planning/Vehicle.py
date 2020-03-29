import math
import numpy as np
from BasicGeometry import BasicGeometry
WIDTH = 1.0
LENGTH = 1.0
class Vehicle:
    # initialise a vehicle, setting its x,y coord and heading
    def __init__(self,x,y,theta):
        self._x = x
        self._y = y
        self._theta = theta
        self._boundaryLeftX = self._x - 0.5*WIDTH
        self._boundaryRightX = self._x + 0.5*WIDTH
        self._boundaryTopY = self._y + 0.5*LENGTH
        self._boundaryBottomY = self._y - 0.5*LENGTH

    @property
    def x(self):
        return self._x
    @property
    def y(self):
        return self._y
    @property
    def theta(self):
        return self._theta

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

    def DistanceTo(self,otherVehicle):
        p1 = (self._x,self._y)
        p2 = (otherVehicle.x,otherVehicle.y)
        euclidean_distance = BasicGeometry.ptDist(p1,p2)
        cosine_distance = 1 - math.cos(math.radians(otherVehicle.theta)-math.radians(self._theta))
        return euclidean_distance + cosine_distance



