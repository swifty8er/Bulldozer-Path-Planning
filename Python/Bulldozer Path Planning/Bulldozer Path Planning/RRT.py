import numpy as np
from enum import Enum
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry
import random
import math
class Status(Enum):
    REACHED = 1
    ADVANCED = 2
    TRAPPED = 3
    EXISTS = 4

class RRT:
    # function to initalise a tree from a start state and a list of control tuples that can be applied
    def __init__(self,map,start_position,controls_list,inverse_control_mappings):
        self._map = map
        if self.testStateCollision(start_position):
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

    def extend(self,x_rand):
        nearest_neighbour = self.nearestNeighbour(x_rand)
        (result,x_new,u_new) = self.generateNewState(x_rand,nearest_neighbour)
        u_inv = self._controls_list[self._inverse_control_mappings[self._controls_list.index(u_new)]]
        if result:
            self.addVertex(x_new)
            (bool1,bool2) = self.addEdge(x_new,nearest_neighbour,u_new,u_inv)
            if bool1 or bool2:
                return Status.EXISTS
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
            dist = node.DistanceTo(x) #distance metric defined in this function
            if dist < min_dist:
                min_dist = dist
                min_node = node

        if min_node == None:
            raise Exception("No nearest neighbour to (%.2f,%.2f,%.2f) found" % (x.x,x.y,x.theta))
        return min_node 

    # given a node and a control that describes following the path of a cirlce arc from the node
    # test if doing so will cause a collision with the obstacles or boundary of the map
    def isCollision(self,node,control):
        edges = self._map.getMapEdges()
        for edge in edges:
            if BasicGeometry.arcLineCollisionAlgorithm(node,control,edge,self._map.disk_radius):
                #print("Collision found between (x,y,theta) = (%.2f,%.2f,%.2f) (r,deltaTheta,direction) = (%.2f,%.2f,%s) edge = (%.2f,%2.f,%.2f,%.2f)" % (node.x,node.y,node.theta,control[0],control[1],control[2],edge[0][0],edge[0][1],edge[1][0],edge[1][1]) )
                return True
        return False


    def testStateCollision(self,node):
        edges = self._map.getMapEdges()
        point = (node.x,node.y)
        for edge in edges:
            if (self._map.disk_radius - BasicGeometry.point2LineDist(edge,point)) > np.finfo(np.float32).eps:
                return True
        return False


    def testMoveCollision(self,node,control):
        edges = self._map.getMapEdges()
        for edge in edges:
            if BasicGeometry.arcLineCollisionIterative(node,control,edge,8,self._map.disk_radius):
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
                dist = x.DistanceTo(x_test)
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
        else:
            self._tree[x_new] = {}
            for key in self._tree.keys():
                self._tree[x_new][key] = False
                self._tree[key][x_new] = False

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



