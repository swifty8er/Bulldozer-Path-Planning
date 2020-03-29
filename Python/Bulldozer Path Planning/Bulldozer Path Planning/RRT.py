from enum import Enum
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry
import random
import math
class Status(Enum):
    REACHED = 1
    ADVANCED = 2
    TRAPPED = 3

class RRT:
    # function to initalise a tree from a start state and a list of control tuples that can be applied
    def __init__(self,map,start_position,controls_list):
        self._tree = self.initaliseTree(start_position)
        self._controls_list = controls_list
        self._map = map

    def initaliseTree(self,start_position):
        tree = {}
        tree[start_position] = {}
        tree[start_position][start_position] = False
        return tree

    def generateRandomState(self):
        randState = self.genRandState()
        while (randState.IsCollidingWithMapBoundaryOrObstacles(self._map)):
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
            self.addVertex(x_new)
            self.addEdge(x_new,nearest_neighbour,u_new)
            if (x_new == x_rand): #overwrite the equality function for vehicles
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
        return False


    def testStateCollision(self,node):
        edges = self._map.getMapEdges()

        point = (node.x,node.y)
        for edge in edges:
            if BasicGeometry.point2LineDist(edge,point) < self._map.disk_radius:
                return True
        return False


    def testMoveCollision(self,node,control):
        edges = self._map.getMapEdges()
        for edge in edges:
            if BasicGeometry.arcLineCollisionIterative(node,control,edge,100,self._map.disk_radius):
                return True
        return False

    # generates a new state from x near in the direction towards x using the available controls
    def generateNewState(self,x,x_near):
        min_dist = math.inf
        x_new = None
        u_new = None
        for control in self._controls_list:
            if (not self.isCollision(x_near,control)):
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

    def addVertex(self,x_new):
        pass # adds the vertex to the tree, no return

    def hasVertex(self,x):
        return ( x in self._tree )

    def addEdge(self,x_new,x_near,u_new):
        pass # add the edge between x_near and x_new to the tree, denoted by u_new, no return



