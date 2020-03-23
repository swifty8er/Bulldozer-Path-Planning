from enum import Enum

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
        return tree

    def generateRandomState(self):
        return None #returns a random vehicle in the state space

    def extend(self,x_rand):
        return None # returns an enum [reached, advanced, trapped]

    def nearestNeighbour(self,x):
        pass # searches the tree for the nearest node to x by some distance metric

    def generateNewState(self,x,x_near):
        pass # generates a new state from x near in the direction towards x using the available controls
        # returns a bool if successful, and the new state and control used

    def addVertex(self,x_new):
        pass # adds the vertex to the tree, no return

    def hasVertex(self,x):
        return ( x in self._tree )

    def addEdge(self,x_new,x_near,u_new):
        pass # add the edge between x_near and x_new to the tree, denoted by u_new, no return



