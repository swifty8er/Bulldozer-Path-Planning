import math
from Vehicle import Vehicle
class Octree:
    def __init__(self,state : Vehicle ,parent ,max_distance,max_size=8,max_levels=5):
        self._centreState = state
        self._max_distance = max_distance
        self._max_size = max_size
        self._max_levels = max_levels
        self._has_children = False
        self._children : Octree = [] #list of Octrees
        self._vehicle_states : Vehicle = [] #list of vehicles
        self._num_states = 0
        self._parent = parent

    @property
    def max_distance(self):
        return self._max_distance

    @property
    def parent(self):
        return self._parent

    @property
    def has_children(self):
        return self._has_children

    @property
    def children(self):
        return self._children

    @property 
    def vehicle_states(self):
        return self._vehicle_states


    @property 
    def num_states(self):
        return self._num_states


    @property
    def centreState(self):
        return self._centreState

    def locateCentreState(self,state):
        if self.centreState == state:
            return True
        else:
            for child in self.children:
                if child.locateCentreState(state):
                    return True
            return False

    def locateState(self,state):
        if not self.has_children and state in self._vehicle_states:
            return (self,True)
        else:
            for child in self.children:
                (node,found) = child.locateState(state)
                if found:
                    return (node,found)
                    break
            return (None,False)



    def extend(self):
        self._has_children = True
        self._children = list([self.generateChildTree(index) for index in range(self._max_size)])


    def getMaxDistanceBetweenVehicleStatesAndCentreState(self):
        max_dist = -1*math.inf
        for vs in self._vehicle_states:
            dist = self._centreState.DistanceMetric(vs)
            if dist > max_dist:
                max_dist = dist
        return max_dist

    def findClosestChildIndex(self, state : Vehicle):
        shortest_dist = math.inf
        closestChildIndex = -1
        i = 0
        for child in self._children:
            dist = state.DistanceMetric(child.centreState)
            if dist < shortest_dist:
                shortest_dist = dist
                closestChildIndex = i
            i+=1
        if closestChildIndex == -1:
            raise Exception("Could not find closest child to state (%.2f,%.2f,%.2f)" % (state.x,state.y,state.theta))
        return closestChildIndex

    def generateChildTree(self,index):
        parentState = self._vehicle_states[index]
        new_max_dist = self.getMaxDistanceBetweenVehicleStatesAndCentreState()
        return Octree(parentState,self,new_max_dist,self._max_size,self._max_levels-1)

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

