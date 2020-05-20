import math
from Vehicle import Vehicle
class Quadtree:
    def __init__(self,state : Vehicle ,parent ,max_distance,max_size=4):
        self._centreState = state
        self._max_distance = max_distance
        self._max_size = max_size
        self._has_children = False
        self._children : Quadtree = [] #list of quadtrees
        self._vehicle_states : Vehicle = [] #list of vehicles
        self._num_states = 0
        self._parent = parent

    @property
    def num_states(self):
        return self._num_states
    
    @property
    def centreState(self):
        return self._centreState
    
    def radialNearestNeighbours(self,queryState,radius,nearest_neighbours):
        if not self._has_children:
            for vs in self._vehicle_states:
                if vs.EuclideanDistance(queryState) < radius:
                    nearest_neighbours.append(vs)
            return nearest_neighbours
        for child in self._children:
            if child.overlapsSearchDisk(queryState,radius):
                nearest_neighbours = child.radialNearestNeighbours(queryState,radius,nearest_neighbours)
        return nearest_neighbours


    def overlapsSearchDisk(self,queryState,radius):
        q_prime = Vehicle(abs(queryState.x-self.centreState.x),abs(queryState.y-self.centreState.y),0)
        new_vec = Vehicle(self._max_distance,self._max_distance,0)
        if q_prime.x < self._max_distance:
            return True
        if q_prime.y < self._max_distance:
            return True
        if q_prime.EuclideanDistance(new_vec) < radius:
            return True
        return False

    def extend(self):
        self._has_children = True
        self._children = list([self.generateChildTree(index) for index in range(self._max_size)])



    def findClosestChildIndex(self, state : Vehicle):
        shortest_dist = math.inf
        closestChildIndex = -1
        i = 0
        for child in self._children:
            dist = state.EuclideanDistance(child.centreState)
            if dist < shortest_dist:
                shortest_dist = dist
                closestChildIndex = i
            i+=1
        if closestChildIndex == -1:
            raise Exception("Could not find closest child to state (%.2f,%.2f,%.2f)" % (state.x,state.y,state.theta))
        return closestChildIndex

    def generateChildTree(self,index):
        parentState = self.getParentState(index)
        return Quadtree(parentState,self,self._max_distance/2.0,self._max_size)


    def getParentState(self,i):
        half_dist = self._max_distance/2.0
        if i == 0:
            return Vehicle(self.centreState.x+half_dist,self.centreState.y+half_dist,0)
        elif i == 1:
            return Vehicle(self.centreState.x+half_dist,self.centreState.y-half_dist,0)
        elif i == 2:
            return Vehicle(self.centreState.x-half_dist,self.centreState.y+half_dist,0)
        elif i == 3:
            return Vehicle(self.centreState.x-half_dist,self.centreState.y-half_dist,0)
        else:
            raise Exception("Invalud index %d passed to get parent state" % (i))

    def addState(self,state : Vehicle):
        if self._has_children:
            self.addStateToChild(state)
        elif len(self._vehicle_states) >= self._max_size:
            self.extend()
            for vs in self._vehicle_states:
                self.addStateToChild(vs)
            self.addStateToChild(state)
            self._vehicle_states = []
        else:
            self._vehicle_states.append(state)
        self._num_states += 1



    def addStateToChild(self,state : Vehicle):
        idx = self.findClosestChildIndex(state)
        self._children[idx].addState(state)



