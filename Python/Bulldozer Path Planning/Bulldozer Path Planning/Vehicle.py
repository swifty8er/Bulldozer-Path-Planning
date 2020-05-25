import math
import bezier
import numpy as np
from BasicGeometry import BasicGeometry
MAX_ANGLE_DIFFERENCE = 5.0
MIN_RADIUS = 0.4
REVERSE_CONTROLS = [((0.4*math.pi)/4.0,0,"R"),(0.4,45,"RR"),(0.4,45,"RL")]
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

    # dummy greater than comparison so that max of k_nn works in parent class
    def __gt__(self, other):
        return False

    def __eq__(self, other):
        if other == None:
            return False
        elif not isinstance(other,Vehicle):
            return False
        else:
            #print("Testing for equality (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)" % (self.x,self.y,self.theta,other.x,other.y,other.theta))
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

    def DistanceMetric(self,otherVehicle,angularWeight=1):
        p1 = (self._x,self._y)
        p2 = (otherVehicle.x,otherVehicle.y)
        euclidean_distance = BasicGeometry.ptDist(p1,p2)
        cosine_distance = 1 - math.cos(math.radians(otherVehicle.theta)-math.radians(self._theta))
        return euclidean_distance + angularWeight * cosine_distance

    def WithinAngleDistanceMetric(self,otherVehicle,tolerance=90):
        if abs(self.theta-otherVehicle.theta)>tolerance:
            return math.inf
        else:
            return self.EuclideanDistance(otherVehicle)

    def EuclideanDistance(self,otherVehicle):
        p1 = (self._x,self._y)
        p2 = (otherVehicle.x,otherVehicle.y)
        euclidean_distance = BasicGeometry.ptDist(p1,p2)
        return euclidean_distance

    #apply reverse and reverse left and right maximum controls to produce new poses
    def getNextPoses(self):
        resulting = []
        for control in REVERSE_CONTROLS:
            resulting_pose = self.applyControl(control[0],control[1],control[2])
            resulting.append((resulting_pose,control))
        return resulting

    def getCircleArcPoints(self,control,num_steps):
        angle = control[1]
        delta_angle = angle/float(num_steps)
        the_angle = delta_angle
        x_points = []
        y_points = []
        for i in range(num_steps+1):
            new_position = self.applyControl(control[0],the_angle,control[2])
            x_points.append(new_position.x)
            y_points.append(new_position.y)
            the_angle += delta_angle
        return (x_points,y_points)

    def isAheadOf(self,otherVehicle):
        line = [[self.x,self.y],[self.x+math.cos(math.radians(self.theta)),self.y+math.sin(math.radians(self.theta))]]
        perp_line = BasicGeometry.getPerpLine(line)
        (m,c,x) = perp_line
        if m == None:
            # line is of form x = 5
            if (self.theta < 180):
                if (otherVehicle.x<x):
                    return True
                else:
                    return False
       
            else:
                if (otherVehicle.x>x):
                    return True
                else:
                    return False
        elif m == 0:
            # line is of the form y = 4
            if (self.theta < 180):
                if (otherVehicle.y<c):
                    return True
                else:
                    return False
            else:
                if (otherVehicle.y>c):
                    return True
                else:
                    return False

        else:
            if (self.theta<180):
                upperBoundY = m*self.x+c
                if otherVehicle.y < upperBoundY:
                    return True
                else:
                    return False
            else:
                lowerBoundY = m*self.x+c
                if otherVehicle.y > lowerBoundY:
                    return True
                else:
                    return False

