class RRT:
    # function to initalise a tree from a start state and a list of control tuples that can be applied
    def __init__(self,start_position,controls_list):
        self._tree = self.initaliseTree(start_position)
        self._controls_list = controls_list

    def initaliseTree(self,start_position):
        pass #returns a dictionary of dictionaries with the vertex start position inserted

    def generateRandomState(self):
        pass #returns a random vehicle in the state space

    def extend(self,x_rand):
        pass # returns an enum [reached, advanced, trapped]

    def nearestNeighbour(self,x):
        pass # searches the tree for the nearest node to x by some distance metric

    def generateNewState(self,x,x_near):
        pass # generates a new state from x near in the direction towards x using the available controls
        # returns a bool if successful, and the new state and control used

    def addVertex(self,x_new):
        pass # adds the vertex to the tree, no return

    def addEdge(self,x_new,x_near,u_new):
        pass # add the edge between x_near and x_new to the tree, denoted by u_new, no return



