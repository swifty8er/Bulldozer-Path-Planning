import numpy as np
import turtle
import bezier
import time
from enum import Enum
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry
import random
import math
from matplotlib import pyplot as plt

SCALING = 100.0
OFFSET = 300.0

class Status(Enum):
    REACHED = 1
    ADVANCED = 2
    TRAPPED = 3
    NODE_EXISTS = 4
    EDGE_EXISTS = 5
    COLLIDING = 6
    BIDIRECTIONAL_SUCCESS = 7

class RRT:
    # function to initalise a tree from a start state and a list of control tuples that can be applied
    def __init__(self,map,start_position,controls_list):
        self._map = map
        self._start_position = start_position
        if self.testStateCollision(start_position): #this test is flawed
            raise Exception("Attempted to initialise the RRT with a colliding state")
        self._tree = self.initaliseTree(start_position)
        self._controls_list = controls_list
    

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
            pen.color("orange")
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
            pen.color("orange")
        (result,x_new,u_new) = self.generateNewState(x_rand,nearest_neighbour)
        u_inv = self.getInverseControl(u_new)
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
          
            if (x_new == x_rand):
                return Status.REACHED
            elif self.edgeCollidesWithDirtPile(nearest_neighbour,x_new,u_new):
                return Status.COLLIDING
            elif nodeExists:
                return Status.NODE_EXISTS
            elif bool1 or bool2:
                return Status.EDGE_EXISTS
            else:
                return Status.ADVANCED

        return Status.TRAPPED # returns an enum [reached, advanced, trapped]

    # Create the inverse control
    def getInverseControl(self,u_new):
        (radius,theta,direction) = u_new
        directionDict = {'F':'R','FL':'RL','FR':'RR','RL':'FL','R':'F','RR':'FR'}
        return (radius,theta,directionDict[direction])


    def getKNearestNeighbours(self,x,k):
        k_nn = []
        for node in self.tree:
            dist = node.DistanceMetric(x,5)
            if len(k_nn)< k:
                k_nn.append((dist,node))
            elif dist<max(k_nn)[0]:
                k_nn.remove(max(k_nn))
                k_nn.append((dist,node))
        nearest_neighbours = [i[1] for i in k_nn]
        return nearest_neighbours

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

    def nodeWithinRadiusOfDirtPile(self,node):
        for pos in self._map.initial_disk_pos_xy:
            if self._map.disk_radius - BasicGeometry.ptDist(pos,(node.x,node.y)) > np.finfo(np.float32).eps:
                return True
        return False

    def edgeCollidesWithDirtPile(self,n1,n2,edge_arc):
        if self.nodeWithinRadiusOfDirtPile(n1):
            return True
        if self.nodeWithinRadiusOfDirtPile(n2):
            return True
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
                    (x1,y1) = p1
                    (x2,y2) = p2
                    if (x1 != None and y1 != None):
                        v1 = BasicGeometry.vec_from_points(circle_2_centre,p1)
                        theta_1 = BasicGeometry.vector_angle(v1)
                    else:
                        theta_1 = None
                    if (x2 != None and y2 != None):
                        v2 = BasicGeometry.vec_from_points(circle_2_centre,p2)
                        theta_2 = BasicGeometry.vector_angle(v2)
                    else:
                        theta_2 = None



                    v3 = BasicGeometry.vec_from_points(circle_2_centre,(n1.x,n1.y))
                    v4 = BasicGeometry.vec_from_points(circle_2_centre,(n2.x,n2.y))

                    theta_3 = BasicGeometry.vector_angle(v3)
                    theta_4 = BasicGeometry.vector_angle(v4)

                    if (theta_4 < theta_3):
                        if (abs(math.degrees(theta_3-theta_4)-deltaTheta)<1):

                            if (theta_1 != None and theta_4 <= theta_1 and theta_1 <= theta_3):
                                print("COLLISION")
                                #print("From (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) edge = (%.4f,%.2f,%s)" % (n1.x,n1.y,n1.theta,n2.x,n2.y,n2.theta,radius,deltaTheta,direction))
                                #print("Circle comparison between (%.2f,%.2f) r1=[%.2f] and (%.2f,%.2f) r2=[%.2f]" % (pos[0],pos[1],self._map.disk_radius,p,q,radius))
                                #print("Intersection points are",x1,y1,x2,y2)
                                #print(math.degrees(theta_4),math.degrees(theta_1),math.degrees(theta_3))
                                return True
                            elif (theta_2 != None and theta_4 <= theta_2 and theta_2 <= theta_3):
                                print("COLLISION")
                                #print("From (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) edge = (%.4f,%.2f,%s)" % (n1.x,n1.y,n1.theta,n2.x,n2.y,n2.theta,radius,deltaTheta,direction))
                                #print("Circle comparison between (%.2f,%.2f) r1=[%.2f] and (%.2f,%.2f) r2=[%.2f]" % (pos[0],pos[1],self._map.disk_radius,p,q,radius))
                                #print("Intersection points are",x1,y1,x2,y2)
                                #print(math.degrees(theta_4),math.degrees(theta_2),math.degrees(theta_3))
                                return True
                        elif (abs(math.degrees(2*math.pi - (theta_3-theta_4))-deltaTheta)<1):
                            if (theta_1 != None and 0<=theta_1 and theta_1<=theta_4):
                                print("COLLISION")
                                return True
                            elif (theta_2 != None and 0<=theta_2 and theta_2<=theta_4):
                                print("COLLISION")
                                return True
                            elif (theta_1 != None and 2*math.pi >= theta_1 and theta_1 >= theta_3):
                                print("COLLISION")
                                return True
                            elif (theta_2 != None and 2*math.pi >= theta_2 and theta_2 >= theta_3):
                                print("COLLISION")
                                return True
                                  
                    else:
                        if (abs(math.degrees(theta_4-theta_3)-deltaTheta)<1):
                            if (theta_1 != None and theta_3 <= theta_1 and theta_1 <= theta_4):
                                print("COLLISION")
                                #print("From (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) edge = (%.4f,%.2f,%s)" % (n1.x,n1.y,n1.theta,n2.x,n2.y,n2.theta,radius,deltaTheta,direction))
                                #print("Circle comparison between (%.2f,%.2f) r1=[%.2f] and (%.2f,%.2f) r2=[%.2f]" % (pos[0],pos[1],self._map.disk_radius,p,q,radius))
                                #print("Intersection points are",x1,y1,x2,y2)
                                #print(math.degrees(theta_3),math.degrees(theta_1),math.degrees(theta_4))
                                return True
                            elif (theta_2 != None and theta_3 <= theta_2 and theta_2 <= theta_4):
                                print("COLLISION")
                                #print("From (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) edge = (%.4f,%.2f,%s)" % (n1.x,n1.y,n1.theta,n2.x,n2.y,n2.theta,radius,deltaTheta,direction))
                                #print("Circle comparison between (%.2f,%.2f) r1=[%.2f] and (%.2f,%.2f) r2=[%.2f]" % (pos[0],pos[1],self._map.disk_radius,p,q,radius))
                                #print("Intersection points are",x1,y1,x2,y2)
                                #print(math.degrees(theta_3),math.degrees(theta_2),math.degrees(theta_4))
                                return True
                        elif (abs(math.degrees(2*math.pi - (theta_4-theta_3))-deltaTheta)<0.1):
                            if (theta_1 != None and 0<= theta_1 and theta_1 <= theta_3):
                                print("COLLISION")
                                return True
                            elif (theta_2 != None and 0<= theta_2 and theta_2 <= theta_3):
                                print("COLLISION")
                                return True
                            elif (theta_1 != None and 2*math.pi>=theta_1 and theta_1 >= theta_4):
                                print("COLLISION")
                                return True
                            elif (theta_2 != None and 2*math.pi>=theta_2 and theta_2 >= theta_4):
                                print("COLLISION")
                                return True


        return False

    def randomiseControlPathLength(self,control):
        (radius,theta,direction) = control
        if direction == 'F' or direction == 'R':
            arcLength = radius
        else:
            arcLength = radius*math.radians(theta)
        newArcLength = random.uniform(arcLength*0.4,arcLength)
        if direction == 'F' or direction == 'R':
            return (newArcLength,theta,direction)
        else:
            newTheta = math.degrees(newArcLength/radius)
            return (radius,newTheta,direction)

    # generates a new state from x near in the direction towards x using the available controls
    def generateNewState(self,x,x_near):
        min_dist = math.inf
        x_new = None
        u_new = None
        for control in self._controls_list:
            newControl = self.randomiseControlPathLength(control)
            if (not self.isCollision(x_near,newControl)):
                #if self.testMoveCollision(x_near,control):
                #    print("Found collision not detected by algorithm from (%.2f,%.2f,%.2f) under control (%.2f,%.2f,%s)" % (x_near.x,x_near.y,x_near.theta,control[0],control[1],control[2]))
                x_test = x_near.applyControl(newControl[0],newControl[1],newControl[2])
                dist = x.DistanceMetric(x_test,1)
                if dist < min_dist: 
                    min_dist = dist
                    x_new = x_test
                    u_new = newControl

        if (x_new == None):
            return (False,Vehicle(0,0,0),(0,0,0)) #dummy second and third values
        else:
            return (True,x_new,u_new) # returns a bool if successful, and the new state and control used

    def addBackwardsVertex(self,x_new,tree):
        if not x_new in tree:
            tree[x_new] = {}
        for key in tree.keys():
            tree[x_new][key] = False
            tree[key][x_new] = False

    # Add the vertex to the tree
    def addVertex(self,x_new):
        if x_new in self._tree:
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

    def addBackwardsEdge(self,x_new,x_near,u_new,u_inv,tree):
        if tree[x_new][x_near] == u_inv or tree[x_near][x_new] == u_new:
            return True
        tree[x_new][x_near] = u_inv
        tree[x_near][x_new] = u_new
        return False

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




    def connectPushPoint(self,push_point):
        backwardsDict = self.populateBackwardsDict(push_point)
        for node in backwardsDict:
            nearest_neighbours = self.getKNearestNeighbours(node,10)
            print("Nearest neighbours to node (%.2f,%.2f,%.2f) are" % (node.x,node.y,node.theta))
            for nn in nearest_neighbours:
                print(nn)
                bezier_new = nn.createBezierCurveControl(node)
                if bezier_new != False:
                    if nn not in backwardsDict:
                        backwardsDict[nn] = {}
                    backwardsDict[nn][node] = bezier_new
                    backwardsDict[node][nn] = False
                    self.tree.update(backwardsDict)
                    return True
        return False
                   



    #def bidirectionalExtend(self,x_rand,x_nn,backwardsDict,pen):
    #    (result,x_new,u_new) = self.generateNewState(x_rand,x_nn)
    #    u_inv = self._controls_list[self._inverse_control_mappings[self._controls_list.index(u_new)]]
    #    # could not find new state
    #    if not result:
    #        return (False,False)
    #    # edge to new state collides with dirt pile
    #    if self.edgeCollidesWithDirtPile(x_nn,x_new,u_new):
    #        return (False,False)
    #    # insert new edge

    #    if pen != None:
    #        pen.up()
    #        pen.goto(x_rand.x*SCALING-OFFSET,x_rand.y*SCALING-OFFSET)
    #        pen.setheading(x_rand.theta)
    #        pen.down()
    #        pen.color("red")
    #        rand_stamp = pen.stamp()
    #        pen.up()
    #        pen.goto(x_nn.x*SCALING-OFFSET,x_nn.y*SCALING-OFFSET)
    #        pen.setheading(x_nn.theta)
    #        pen.color("green")
    #        pen.down()
    #        nn_stamp = pen.stamp()
    #        pen.color("orange")
    #        pen.dot(4)
    #        pen.forward(SCALING/10)
    #        pen.left(145)
    #        pen.forward(SCALING/20)
    #        pen.back(SCALING/20)
    #        pen.right(145)
    #        pen.right(145)
    #        pen.forward(SCALING/20)
    #        pen.back(SCALING/20)
    #        pen.left(145)
    #        pen.back(SCALING/10)
    #        pen.color("orange")
    #        (radius,dTheta,direction) = u_new
    #        if direction == "F":
    #            pen.forward(radius*SCALING)
    #        elif direction == "R":
    #            pen.back(radius*SCALING)
    #        elif (direction == "FL"):
    #            pen.circle(radius*SCALING,dTheta)
    #        elif (direction == "RL"):
    #            pen.circle(radius*SCALING,-1*dTheta)
    #        else:
    #            pen.left(180)
    #            if (direction == "RR"):
    #                pen.circle(radius*SCALING,dTheta)
    #            else:
    #                pen.circle(radius*SCALING,-1*dTheta)
    #            pen.left(180)
    #        pen.dot(4)
    #        pen.forward(SCALING/10)
    #        pen.left(145)
    #        pen.forward(SCALING/20)
    #        pen.back(SCALING/20)
    #        pen.right(145)
    #        pen.right(145)
    #        pen.forward(SCALING/20)
    #        pen.back(SCALING/20)
    #        pen.left(145)
    #        pen.back(SCALING/10)
    #        pen.up()
    #        pen.clearstamp(rand_stamp)
    #        pen.clearstamp(nn_stamp)
    #    self.addBackwardsVertex(x_new,backwardsDict)
    #    if self.addBackwardsEdge(x_new,x_nn,u_new,u_inv,backwardsDict):
    #        return (False,False)
    #    elif x_new in self._tree:
    #        return (True,True) #bidirectional growth meets old RRT
    #    else:
    #        return (True,False) #worked but algorithm not finished


    #def findNearestNeighbour(self,x_rand,dictionary):
    #    min_dist = math.inf
    #    min_node = None
    #    for node in dictionary.keys():
    #        dist = node.EuclideanDistance(x_rand)
    #        if dist < min_dist:
    #            min_dist = dist
    #            min_node = node

    #    if min_node == None:
    #        raise Exception("No nearest neighbour to (%.2f,%.2f,%.2f) found" % (x.x,x.y,x.theta))
    #    return min_node 


    #    randState = self.genRandState()
    #    while (self.testStateCollision(randState)):
    #        randState = self.genRandState()

    #    return randState #returns a random vehicle in the state space

    #    new_x = random.uniform(self._map.min_x,self._map.max_x)
    #    new_y = random.uniform(self._map.min_y,self._map.max_y)
    #    new_theta = random.uniform(0,360)
    #    return Vehicle(new_x,new_y,new_theta)

    #def generateBehindState(self,startingNode):
    #    line = [[startingNode.x,startingNode.y],[startingNode.x+math.cos(math.radians(startingNode.theta)),startingNode.y+math.sin(math.radians(startingNode.theta))]]
    #    perp_line = BasicGeometry.getPerpLine(line)
    #    (m,c,x) = perp_line
    #    if m == None:
    #        # line is of form x = 5
    #        if (startingNode.theta < 180):
    #            new_x = random.uniform(self._map.min_x,x)
       
    #        else:
    #            new_x = random.uniform(x,self._map.max_x)

    #        new_y = random.uniform(self._map.min_y,self._map.max_y)
    #        new_theta = random.uniform(0,360)
    #        newVehicle = Vehicle(new_x,new_y,new_theta)
    #        if self.testStateCollision(newVehicle):
    #            return self.generateBehindState(startingNode)
    #        else:
    #            return newVehicle
    #    elif m == 0:
    #        # line is of the form y = 4
    #        if (startingNode.theta < 180):
    #            new_y = random.uniform(self._map.min_y,c)
    #        else:
    #            new_y = random.uniform(c,self._map.max_y)
    #        new_x = random.uniform(self._map.min_x,self._map.max_x)
    #        new_theta = random.uniform(0,360)
    #        newVehicle = Vehicle(new_x,new_y,new_theta)
    #        if self.testStateCollision(newVehicle):
    #            return self.generateBehindState(startingNode)
    #        else:
    #            return newVehicle

    #    else:
    #        new_x = random.uniform(self._map.min_x,self._map.max_x)
    #        if (startingNode.theta<180):
    #            upperBoundY = m*new_x+c
    #            if upperBoundY<=self._map.min_y:
    #                return self.generateBehindState(startingNode)
    #            else:
    #                new_y = random.uniform(self._map.min_y,upperBoundY)
    #        else:
    #            lowerBoundY = m*new_x+c
    #            if lowerBoundY >= self._map.max_y:
    #                return self.generateBehindState(startingNode)
    #            else:
    #                new_y = random.uniform(lowerBoundY,self._map.max_y)

    #        new_theta = random.uniform(0,360)
    #        newVehicle = Vehicle(new_x,new_y,new_theta)
    #        if self.testStateCollision(newVehicle):
    #            return self.generateBehindState(startingNode)
    #        else:
    #            return newVehicle

    def populateBackwardsDict(self,startingNode):
        backwardsDict = {}
        backwardsDict[startingNode] = {}
        backwardsDict[startingNode][startingNode] = False
        for control in self._controls_list:
            newControl = self.randomiseControlPathLength(control)
            (radius,deltaTheta,direction) = newControl
            if direction == "RL" or direction == "RR" or direction == "R":
                if not (self.isCollision(startingNode,newControl)):
                    newNode = startingNode.applyControl(radius,deltaTheta,direction)
                    u_inv = self.getInverseControl(newControl)
                    if newNode not in backwardsDict:
                        backwardsDict[newNode] = {}
                    backwardsDict[newNode][startingNode] = u_inv
                    backwardsDict[startingNode][newNode] = control

        return backwardsDict
    
    def connectNodesStraightLinePath(self,n1,n2):
        edge = False
        if (math.cos(math.radians(n1.theta)) == 0):
            if round(n1.x,2) == round(n2.x,2):
                if n1.theta == 90:
                    if n2.y > n1.y:
                        edge = (n2.y-n1.y,0,'F')
                    else:
                        edge = (n1.y-n2.y,0,'R')
                elif n1.theta == 270:
                    if n2.y>n1.y:
                        edge = (n2.y-n1.y,0,'R')
                    else:
                        edge = (n1.y-n2.y,0,'F')
                else:
                    raise Exception("Cosine was zero but angle did not match")
        elif (math.sin(math.radians(n1.theta)) == 0):
            if round(n1.y,2) == round(n2.y,2):
                if n1.theta == 0:
                    if n2.x > n1.x:
                        edge = (n2.x-n1.x,0,'F')
                    else:
                        edge = (n1.x-n2.x,0,'R')
                elif n1.theta == 180:
                    if n2.x > n1.x:
                        edge = (n2.x-n1.x,0,'R')
                    else:
                        edge = (n1.x-n2.x,0,'F')
                else:
                    raise Exception("Sine was zero but angle did not match")
        else:
            k_alpha = (n2.x-n1.x)/math.cos(math.radians(n1.theta))
            k_beta = (n2.y-n1.y)/math.sin(math.radians(n1.theta))

            if (k_alpha == k_beta):
                if k_alpha >= 0:
                    edge = (BasicGeometry.ptDist([n1.x,n1.y],[n2.x,n2.y]),0,'F')
                else:
                    edge = (BasicGeometry.ptDist([n1.x,n1.y],[n2.x,n2.y]),0,'R')

        return edge








    def draw(self,ax):
        for n1 in self.tree.keys():
            for n2 in self.tree[n1].keys():
                edge = self.tree[n1][n2]
                if isinstance(edge,bezier.curve.Curve):
                    edge.plot(100,color=[235.0/255.0,131.0/255.0,52.0/255.0],ax=ax)
                elif edge != False:
                    try:
                        (radius,theta,direction) = edge
                    except:
                        raise Exception("Invalid RRT edge found")
                    if direction == 'F' or direction == 'R':
                        ax.plot([n1.x,n2.x],[n1.y,n2.y],'k-',linewidth=1)
                        #print("Plotted line")
                        #plt.show(block=False)
                    else:
                        (x_points,y_points) = n1.getCircleArcPoints(edge,25)
                        ax.plot(x_points,y_points,'k-',linewidth=1)
                        #print("Plotted other control")
                        #plt.show(block=False)


    #def draw(self,pen,scaling,offset):
    #    for node in self.tree.keys():
    #        pen.up()
    #        pen.goto(node.x*scaling-offset,node.y*scaling-offset)
    #        pen.setheading(node.theta)
    #        pen.down()
    #        for n2 in self.tree[node].keys():
    #            if (self.tree[node][n2] in self._controls_list):
    #                #if (MyRRT.edgeCollidesWithDirtPile(node,n2,self.tree[node][n2])):
    #                #    pen.color("red")
    #                #else:
    #                #    pen.color("black")
    #                pen.up()
    #                pen.goto(n2.x*scaling-offset,n2.y*scaling-offset)
    #                pen.down()
    #                pen.dot(4)
    #                pen.setheading(n2.theta)
    #                pen.forward(scaling/10)
    #                pen.left(145)
    #                pen.forward(scaling/20)
    #                pen.back(scaling/20)
    #                pen.right(145)
    #                pen.right(145)
    #                pen.forward(scaling/20)
    #                pen.back(scaling/20)
    #                pen.left(145)
    #                pen.back(scaling/10)
    #                pen.up()
    #                pen.goto(node.x*scaling-offset,node.y*scaling-offset)
    #                pen.setheading(node.theta)
    #                pen.down()
    #                pen.dot(4)
    #                pen.forward(scaling/10)
    #                pen.left(145)
    #                pen.forward(scaling/20)
    #                pen.back(scaling/20)
    #                pen.right(145)
    #                pen.right(145)
    #                pen.forward(scaling/20)
    #                pen.back(scaling/20)
    #                pen.left(145)
    #                pen.back(scaling/10)
    #                (radius,dTheta,direction) = self.tree[node][n2]
    #                if direction == "F":
    #                    pen.forward(radius*scaling)
    #                elif direction == "R":
    #                    pen.back(radius*scaling)
    #                elif (direction == "FL"):
    #                    pen.circle(radius*scaling,dTheta)
    #                elif (direction == "RL"):
    #                    pen.circle(radius*scaling,-1*dTheta)
    #                else:
    #                    #flag = True
    #                    pen.left(180)
    #                    if (direction == "RR"):
    #                        pen.circle(radius*scaling,dTheta)
    #                    else:
    #                        pen.circle(radius*scaling,-1*dTheta)
    #                pen.up()
    #                pen.goto(node.x*scaling-offset,node.y*scaling-offset)
    #                pen.setheading(node.theta)
    #                pen.down()






