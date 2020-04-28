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
NUM_NODES = 3000

class Status(Enum):
    REACHED = 1
    ADVANCED = 2
    TRAPPED = 3
    NODE_EXISTS = 4
    EDGE_EXISTS = 5

class RRT:
    # function to initalise a tree from a start state and a list of control tuples that can be applied
    def __init__(self,map,start_position,controls_list,num_nodes):
        self._map = map
        self._start_position = start_position
        if self.testStateCollision(start_position): #this test is flawed
            raise Exception("Attempted to initialise the RRT with a colliding state")
        self._tree = self.initaliseTree(start_position)
        self._controls_list = controls_list
        self._num_nodes = num_nodes
    
    @property
    def num_nodes(self):
        return self._num_nodes

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

    def extend(self,x_rand):
        nearest_neighbour = self.nearestNeighbour(x_rand)
        (result,x_new,u_new) = self.generateNewState(x_rand,nearest_neighbour)
        u_inv = self.getInverseControl(u_new)
        if result:
            nodeExists = self.addVertex(x_new)
            (bool1,bool2) = self.addEdge(x_new,nearest_neighbour,u_new,u_inv)
            if (x_new == x_rand):
                return Status.REACHED
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
        if direction not in directionDict:
            print("Error direction not found",direction)
        return (radius,theta,directionDict[direction])

    # get the nearest neighbours that are behind the push point
    def getNearestNeighboursToPushPoint(self,push_point,k):
        k_nn = []
        for node in self.tree:
            if push_point.isAheadOf(node):
                dist = push_point.WithinAngleDistanceMetric(node)
                if dist == math.inf:
                    continue
                if isinstance(dist,Vehicle) or isinstance(max(k_nn)[0],Vehicle):
                    print("Error found Vehicles in nearest neighbours to push point")
                    print(dist)
                    print(max(k_nn)[0])
                    continue
                if len(k_nn) < k:
                    k_nn.append((dist,node))
                elif dist < max(k_nn)[0]:
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
                                return True
                            elif (theta_2 != None and theta_4 <= theta_2 and theta_2 <= theta_3):
                                return True
                        elif (abs(math.degrees(2*math.pi - (theta_3-theta_4))-deltaTheta)<1):
                            if (theta_1 != None and 0<=theta_1 and theta_1<=theta_4):
                                return True
                            elif (theta_2 != None and 0<=theta_2 and theta_2<=theta_4):
                                return True
                            elif (theta_1 != None and 2*math.pi >= theta_1 and theta_1 >= theta_3):
                                return True
                            elif (theta_2 != None and 2*math.pi >= theta_2 and theta_2 >= theta_3):
                                return True
                                  
                    else:
                        if (abs(math.degrees(theta_4-theta_3)-deltaTheta)<1):
                            if (theta_1 != None and theta_3 <= theta_1 and theta_1 <= theta_4):
                                return True
                            elif (theta_2 != None and theta_3 <= theta_2 and theta_2 <= theta_4):
                                return True
                        elif (abs(math.degrees(2*math.pi - (theta_4-theta_3))-deltaTheta)<0.1):
                            if (theta_1 != None and 0<= theta_1 and theta_1 <= theta_3):
                                return True
                            elif (theta_2 != None and 0<= theta_2 and theta_2 <= theta_3):
                                return True
                            elif (theta_1 != None and 2*math.pi>=theta_1 and theta_1 >= theta_4):
                                return True
                            elif (theta_2 != None and 2*math.pi>=theta_2 and theta_2 >= theta_4):
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



    # Make the maximum number of connections between the push_point and the reversed positions and their nearest neighbours
    def connectPushPoint(self,push_point,axis=False):
        backwardsDict = self.populateBackwardsDict(push_point)
        connected = False
        nodes = list(backwardsDict.keys()).copy()
        for node in nodes:
            nearest_neighbours = self.getNearestNeighboursToPushPoint(node,int(self.num_nodes/10))
            for nn in nearest_neighbours:
                bezier_new = nn.createBezierCurveControl(node)
                if bezier_new != False:
                    if nn not in backwardsDict:
                        backwardsDict[nn] = {}
                    backwardsDict[nn][node] = bezier_new
                    backwardsDict[node][nn] = False
                    if axis!=False:
                        bezier_new.plot(100,color=[235.0/255.0,131.0/255.0,52.0/255.0],ax=axis)
                    connected = True

        self.tree.update(backwardsDict)
        return connected
                   


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
                    else:
                        (x_points,y_points) = n1.getCircleArcPoints(edge,25)
                        ax.plot(x_points,y_points,'k-',linewidth=1)








