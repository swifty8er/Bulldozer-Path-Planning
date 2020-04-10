import numpy as np
import turtle
from enum import Enum
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry
import random
import math

SCALING = 100.0
OFFSET = 300.0

class Status(Enum):
    REACHED = 1
    ADVANCED = 2
    TRAPPED = 3
    NODE_EXISTS = 4
    EDGE_EXISTS = 5

class RRT:
    # function to initalise a tree from a start state and a list of control tuples that can be applied
    def __init__(self,map,start_position,controls_list,inverse_control_mappings):
        self._map = map
        self._start_position = start_position
        if self.testStateCollision(start_position): #this test is flawed
            raise Exception("Attempted to initialise the RRT with a colliding state")
        self._tree = self.initaliseTree(start_position)
        self._controls_list = controls_list
        self._inverse_control_mappings = inverse_control_mappings

    @property
    def tree(self):
        return self._tree
    
    def initaliseTree(self,start_position):
        tree = {}
        tree[start_position] = {}
        tree[start_position][start_position] = False
        return tree

    def generateRandomState(self):
        randState = self.genRandState()
        while (self.testStateCollision(randState)):
            randState = self.genRandState()

        return randState #returns a random vehicle in the state space

    def genRandState(self):
        new_x = random.uniform(self._map.min_x,self._map.max_x)
        new_y = random.uniform(self._map.min_y,self._map.max_y)
        new_theta = random.uniform(0,360)
        return Vehicle(new_x,new_y,new_theta)

    def extend(self,x_rand,pen):
        nearest_neighbour = self.nearestNeighbour(x_rand)
        if pen != None:
            pen.up()
            pen.goto(x_rand.x*SCALING-OFFSET,x_rand.y*SCALING-OFFSET)
            pen.setheading(x_rand.theta)
            pen.down()
            pen.color("red")
            rand_stamp = pen.stamp()
            pen.up()
            pen.goto(nearest_neighbour.x*SCALING-OFFSET,nearest_neighbour.y*SCALING-OFFSET)
            pen.setheading(nearest_neighbour.theta)
            pen.color("green")
            pen.down()
            nn_stamp = pen.stamp()
            pen.color("black")
            pen.dot(4)
            pen.forward(SCALING/10)
            pen.left(145)
            pen.forward(SCALING/20)
            pen.back(SCALING/20)
            pen.right(145)
            pen.right(145)
            pen.forward(SCALING/20)
            pen.back(SCALING/20)
            pen.left(145)
            pen.back(SCALING/10)
            pen.color("black")
        (result,x_new,u_new) = self.generateNewState(x_rand,nearest_neighbour)
        u_inv = self._controls_list[self._inverse_control_mappings[self._controls_list.index(u_new)]]
        if result:
            nodeExists = self.addVertex(x_new)
            (bool1,bool2) = self.addEdge(x_new,nearest_neighbour,u_new,u_inv)
            if pen != None:
                (radius,dTheta,direction) = u_new
                if direction == "F":
                    pen.forward(radius*SCALING)
                elif direction == "R":
                    pen.back(radius*SCALING)
                elif (direction == "FL"):
                    pen.circle(radius*SCALING,dTheta)
                elif (direction == "RL"):
                    pen.circle(radius*SCALING,-1*dTheta)
                else:
                    pen.left(180)
                    if (direction == "RR"):
                        pen.circle(radius*SCALING,dTheta)
                    else:
                        pen.circle(radius*SCALING,-1*dTheta)
                    pen.left(180)
                pen.dot(4)
                pen.forward(SCALING/10)
                pen.left(145)
                pen.forward(SCALING/20)
                pen.back(SCALING/20)
                pen.right(145)
                pen.right(145)
                pen.forward(SCALING/20)
                pen.back(SCALING/20)
                pen.left(145)
                pen.back(SCALING/10)
                pen.up()
                pen.clearstamp(rand_stamp)
                pen.clearstamp(nn_stamp)
            if nodeExists:
                return Status.NODE_EXISTS
            elif bool1 or bool2:
                return Status.EDGE_EXISTS
            elif (x_new == x_rand): #overwrite the equality function for vehicles
                return Status.REACHED
            else:
                return Status.ADVANCED

        return Status.TRAPPED # returns an enum [reached, advanced, trapped]

    # searches the tree for the nearest node to x by some distance metric
    def nearestNeighbour(self,x):
        min_dist = math.inf
        min_node = None
        for node in self._tree.keys():
            dist = node.EuclideanDistance(x)
            if dist < min_dist:
                min_dist = dist
                min_node = node

        if min_node == None:
            raise Exception("No nearest neighbour to (%.2f,%.2f,%.2f) found" % (x.x,x.y,x.theta))
        return min_node 

    # given a node and a control that describes following the path of a cirlce arc from the node
    # test if doing so will cause a collision with the obstacles or boundary of the map
    def isCollision(self,node,control):
        edges = self._map.getMapEdgesAndObstacles()
        for edge in edges:
            if BasicGeometry.arcLineCollisionAlgorithm(node,control,edge,self._map.disk_radius):
                #print("Collision found between (x,y,theta) = (%.2f,%.2f,%.2f) (r,deltaTheta,direction) = (%.2f,%.2f,%s) edge = (%.2f,%2.f,%.2f,%.2f)" % (node.x,node.y,node.theta,control[0],control[1],control[2],edge[0][0],edge[0][1],edge[1][0],edge[1][1]) )
                return True
        return False


    def testStateCollision(self,node):
        edges = self._map.getMapEdgesAndObstacles()
        point = (node.x,node.y)
        line = [(self._start_position.x,self._start_position.y),point]
        for edge in edges:
            if (self._map.disk_radius - BasicGeometry.point2LineDist(edge,point)) > np.finfo(np.float32).eps:
                return True
            elif (BasicGeometry.doLinesIntersect(edge,line)):
                return True
        return False


    def testMoveCollision(self,node,control):
        edges = self._map.getMapEdgesAndObstacles()
        for edge in edges:
            if BasicGeometry.arcLineCollisionIterative(node,control,edge,8,self._map.disk_radius):
                return True
        return False


    def edgeCollidesWithDirtPile(self,n1,n2):
        edge_arc = self.tree[n1][n2]
        if (edge_arc != False):
            (radius,deltaTheta,direction) = edge_arc
            if direction == "F" or direction == "R":
                line = [[n1.x,n1.y],[n2.x,n2.y]]
                for pos in self._map.initial_disk_pos_xy:
                    if BasicGeometry.doesCircleIntersectLine(pos,self._map.disk_radius,line):
                        return True
            else:
                if direction == "FL" or direction == "RL":
                    p = n1.x + radius*math.cos(math.radians(n1.theta)+math.pi/2)
                    q = n1.y + radius*math.sin(math.radians(n1.theta)+math.pi/2)
                else:
                    p = n1.x + radius*math.cos(math.radians(n1.theta)-math.pi/2)
                    q = n1.y + radius*math.sin(math.radians(n1.theta)-math.pi/2)
                circle_2_centre = (p,q)
                for pos in self._map.initial_disk_pos_xy:
                    (p1,p2) = BasicGeometry.twoCirclesIntersectionPoints(self._map.disk_radius,pos,radius,circle_2_centre)
                    print("Intersection points of",pos,self._map.disk_radius,"and ",circle_2_centre,radius)
                    print("Are:",p1,p2)
                    (x1,y1) = p1
                    (x2,y2) = p2
                    if (x1 != None and x2 != None and y1 != None and y2 !=None):
                        theta_1 = math.acos((x1-p)/radius)
                        if theta_1<0:
                            theta_1 = 2*math.pi - theta_1

                        theta_2 = math.acos((x2-p)/radius)
                        if theta_2 < 0:
                            theta_2 = 2*math.pi - theta_2

                        theta_3_deg = n1.theta + 90
                        theta_4_deg = n2.theta + 90
                        theta_3 = math.radians(theta_3_deg)
                        theta_4 = math.radians(theta_4_deg)
                        if (theta_1 < theta_2):
                            if (theta_1 <= theta_3 and theta_3 <= theta_2):
                                return True
                            elif (theta_1 <= theta_4 and theta_4 <= theta_2):
                                return True
                        else:
                            if (theta_2 <= theta_3 and theta_3 <= theta_1):
                                return True
                            elif (theta_2 <= theta_4 and theta_3 <= theta_1):
                                return True

        return False

    # generates a new state from x near in the direction towards x using the available controls
    def generateNewState(self,x,x_near):
        min_dist = math.inf
        x_new = None
        u_new = None
        for control in self._controls_list:
            if (not self.isCollision(x_near,control)):
                #if self.testMoveCollision(x_near,control):
                #    print("Found collision not detected by algorithm from (%.2f,%.2f,%.2f) under control (%.2f,%.2f,%s)" % (x_near.x,x_near.y,x_near.theta,control[0],control[1],control[2]))
                x_test = x_near.applyControl(control[0],control[1],control[2])
                dist = x.DistanceMetric(x_test)
                if dist < min_dist: 
                    min_dist = dist
                    x_new = x_test
                    u_new = control

        if (x_new == None):
            return (False,Vehicle(0,0,0),(0,0,0)) #dummy second and third values
        else:
            return (True,x_new,u_new) # returns a bool if successful, and the new state and control used

    # Add the vertex to the tree, no return
    def addVertex(self,x_new):
        if x_new in self._tree.keys():
            print("Adding vertex to tree that already exists (%.2f,%.2f,%.2f)" % (x_new.x,x_new.y,x_new.theta))
            return True
        else:
            self._tree[x_new] = {}
            for key in self._tree.keys():
                self._tree[x_new][key] = False
                self._tree[key][x_new] = False
            return False

    def hasVertex(self,x):
        return ( x in self._tree )

    #u_new from x_near to x_new
    #u_inv from x_new to x_near
    def addEdge(self,x_new,x_near,u_new,u_inv):
        bool1 = False
        bool2 = False
        if (x_new == x_near):
            print("Cannot insert an edge from a node to itself")
            return (True,True)
        if x_new in self._tree.keys():
            if x_near in self._tree[x_new].keys():
                if (self._tree[x_new][x_near] == False):
                    self._tree[x_new][x_near] = u_inv
                elif (self._tree[x_new][x_near] == u_inv):
                    bool1 = True
                else:
                    print("New edge between states?? (should overwrite?)")
            else:
                raise Exception("x_near not in tree under key x_new")
        else:
            raise Exception("x_new not in tree")

        if x_near in self._tree.keys():
            if x_new in self._tree[x_near].keys():
                if (self._tree[x_near][x_new] == False):
                    self._tree[x_near][x_new] = u_new
                elif (self._tree[x_near][x_new] == u_new):
                    bool2 = True
                else:
                    print("New edge between states?? (should overwrite?)")
            else:
                raise Exception("x_new not in tree under key x_near")
        else:
            raise Exception("x_near not in tree")

        return (bool1,bool2)



