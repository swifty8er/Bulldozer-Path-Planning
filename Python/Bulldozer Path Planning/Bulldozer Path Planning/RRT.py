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
    EDGE_EXISTS = 4

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
        self._quadtree = None
        self._cachedNearestNeighbours = {}
    
    @property
    def num_nodes(self):
        return self._num_nodes

    @property
    def tree(self):
        return self._tree

    def setQuadtree(self,quadtree):
        self._quadtree = quadtree
    
    def initaliseTree(self,start_position):
        tree = {}
        tree[start_position] = {}
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
        if result:
            u_inv = self.getInverseControl(u_new) 
            (bool1,bool2) = self.addEdge(x_new,nearest_neighbour,u_new,u_inv)
            if (x_new == x_rand):
                return Status.REACHED
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


    def addBehindStates(self,push_point,states):
        behindStates = []
        for state in states:
            if push_point.isAheadOf(state):
                behindStates.append(state)
        return behindStates




    def computeMaxDistanceMetricBetweenNodes(self,centre_node):
        max_dist = -1*math.inf
        for node in self._tree:
            dist = centre_node.DistanceMetric(node)
            if dist > max_dist:
                max_dist = dist
        return max_dist
    

    def computeMaxDistanceBetweenNodes(self,centre_node):
        max_dist = -1*math.inf
        for node in self._tree:
            dist = centre_node.EuclideanDistance(node)
            if dist > max_dist:
                max_dist = dist

        return max_dist


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

    def nodeWithinRadiusOfDirtPile(self,node,dirt_pile_positions):
        for pos in dirt_pile_positions:
            if 1.95 * self._map.disk_radius - BasicGeometry.ptDist(pos,(node.x,node.y)) > np.finfo(np.float32).eps:
                return True
        return False

    def pointWithinRadiusOfDirtPile(self,point,dirt_pile_positions):
        for pos in dirt_pile_positions:
            if 1.95 * self._map.disk_radius - BasicGeometry.ptDist(pos,point) > np.finfo(np.float32).eps:
                return True
        return False
  
    def edgeCollidesWithDirtPile(self,n1,n2,edge_arc,dirt_pile_positions):
        if self.nodeWithinRadiusOfDirtPile(n1,dirt_pile_positions):
            return True
        if self.nodeWithinRadiusOfDirtPile(n2,dirt_pile_positions):
            return True
        if (edge_arc != False):
            if isinstance(edge_arc[0],bezier.curve.Curve):
                (bezierEdge,direction) = edge_arc
                s = 0.0
                while s<1.0:
                    point = bezierEdge.evaluate(s)
                    if self.pointWithinRadiusOfDirtPile(point,dirt_pile_positions):
                        return True
                    s+= 0.02

                return False

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
            randNum = random.randint(1,10)
            if randNum<5:
                newControl = self.randomiseControlPathLength(control)
            else:
                newControl = control
            if (not self.isCollision(x_near,newControl)):
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
        if x_new in self._tree:
            if x_near in self._tree[x_new]:
                if (self._tree[x_new][x_near] == u_inv):
                    bool1 = True
                else:
                    self._tree[x_new][x_near] = u_inv
            else:
                self._tree[x_new][x_near] = u_inv
        else:
            self._tree[x_new] = {}
            self._tree[x_new][x_near] = u_inv

        if x_near in self._tree:
            if x_new in self._tree[x_near]:
                if (self._tree[x_near][x_new] == u_new):
                    bool2 = True
                else:
                    self._tree[x_near][x_new] = u_new
            else:
               self._tree[x_near][x_new] = u_new
        else:
            self._tree[x_near] = {}
            self._tree[x_near][x_new] = u_new
        

        return (bool1,bool2)


    def bezierEdgeObstaclesCollision(self,bezierCurve):
        s = 0.0
        edges = self._map.getMapEdgesAndObstacles()
        while s<=1.0:
            point_list = bezierCurve.evaluate(s).tolist()
            point = [i[0] for i in point_list]
            for edge in edges:
                if self._map.disk_radius - BasicGeometry.point2LineDist(edge,point) > np.finfo(np.float32).eps:
                    return True
            s+=0.02

        return False


    # Make the maximum number of connections between the push_point and its reversed extreme control nodes and their nearest neighbours
    def connectPushPoint(self,push_point,curr_disk_pos,axis=False):
        print("Connecting to push point...")
        if push_point in self.tree: #if push point is already connected to tree, return true
            print("Done")
            return True
        pos_tuple = (curr_disk_pos[0],curr_disk_pos[1])
        nearest_neighbours = None
        for key in self._cachedNearestNeighbours:
            if BasicGeometry.ptDist(key,pos_tuple) < 0.4:
                nearest_neighbours = self._cachedNearestNeighbours[key]
                break
       
        if nearest_neighbours == None:
            nearest_neighbours = self._quadtree.radialNearestNeighbours(push_point,1.75,[])
        self._cachedNearestNeighbours[pos_tuple] = nearest_neighbours


        connected = False
        backwardsNodes = self.enumerateBackwardsControls(push_point) #create the two extreme reverse control points
        for node in backwardsNodes:
            processed_nn = self.postProcessNearestNeighbours(node,nearest_neighbours)
            for nn in processed_nn:
                bezier_new = BezierLib.createBezierCurveBetweenTwoVehicle(nn,node)
                if bezier_new != False:
                    if not self.bezierEdgeObstaclesCollision(bezier_new):
                        self.addEdge(node,nn,(bezier_new,"F"),(bezier_new,"R"))
                        if axis!=False:
                            bezier_new.plot(100,color=[235.0/255.0,131.0/255.0,52.0/255.0],ax=axis)
                        connected = True 

                    
        print("Done")
        return connected
     
    #perform post processing on radial nearest neighbours
    def postProcessNearestNeighbours(self,push_point,nearest_neighbours):
        nn = self.addBehindStates(push_point,nearest_neighbours)
        processed_nn = []
        for node in nn:
            if push_point.EuclideanDistance(node) > 0.25:
                if (1-math.cos(math.radians(abs(node.theta-push_point.theta)))) < 1.5:
                    processed_nn.append(node)
        return processed_nn


    def enumerateBackwardsControls(self,startingNode):
        backwardsNodes = [startingNode]
        cI = [17,33]
        for i in cI:
            control = self._controls_list[i]
            (radius,deltaTheta,direction) = control
            if not (self.isCollision(startingNode,control)):
                newNode = startingNode.applyControl(radius,deltaTheta,direction)
                u_inv = self.getInverseControl(control)
                self.addEdge(newNode,startingNode,control,u_inv)
                backwardsNodes.append(newNode)
        return backwardsNodes




    def drawEdge(self,n1,n2,axis,c):   
        if n1 in self.tree:
            if n2 in self.tree[n1]:
                edge = self.tree[n1][n2]
                if c == 'k-':
                    bezierColor = [0.0,0.0,0.0]
                else:
                    bezierColor = [235.0/255.0,131.0/255.0,52.0/255.0]
                if isinstance(edge[0],bezier.curve.Curve):
                    edge[0].plot(100,color=bezierColor,ax=axis)
                else:
                    try:
                        (radius,theta,direction) = edge
                    except:
                        raise Exception("Invalid RRT edge found")
                    if direction == 'F' or direction == 'R':
                        axis.plot([n1.x,n2.x],[n1.y,n2.y],c,linewidth=1)
                    else:
                        (x_points,y_points) = n1.getCircleArcPoints(edge,25)
                        axis.plot(x_points,y_points,c,linewidth=1)
            else:
                print("Edge from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) does not exist" % (n1.x,n1.y,n1.theta,n2.x,n2.y,n2.theta))
        else:
            print("Edge from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) does not exist" % (n1.x,n1.y,n1.theta,n2.x,n2.y,n2.theta))


    def draw(self,ax):
        for n1 in self.tree.keys():
            for n2 in self.tree[n1].keys():
                edge = self.tree[n1][n2]
                if isinstance(edge[0],bezier.curve.Curve):
                    edge.plot(100,color=[235.0/255.0,131.0/255.0,52.0/255.0],ax=ax)
                else:
                    try:
                        (radius,theta,direction) = edge
                    except:
                        raise Exception("Invalid RRT edge found")
                    if direction == 'F' or direction == 'R':
                        ax.plot([n1.x,n2.x],[n1.y,n2.y],'k-',linewidth=1)
                    else:
                        (x_points,y_points) = n1.getCircleArcPoints(edge,25)
                        ax.plot(x_points,y_points,'k-',linewidth=1)








