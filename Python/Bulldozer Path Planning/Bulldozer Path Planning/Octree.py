import math
from Vehicle import Vehicle
class Octree:
    def __init__(self,state : Vehicle ,max_distance,max_size=8):
        self._centreState = state
        self._max_distance = max_distance
        self._max_size = max_size
        self._has_children = False
        self._children : Octree = [] #list of Octrees
        self._vehicle_states : Vehicle = [] #list of vehicles
        self._num_states = 0

    def extend(self):
        self._has_children = True
        self._children = list([self.generateChildTree(index) for index in range(self._max_size)])


    def getMaxDistanceBetweenVehicleStatesAndCentreState(self):


    def generateChildTree(self,index):
        parentState = self.__vehicle_states[index]
        new_max_dist = self.getMaxDistanceBetweenVehicleStatesAndCentreState()
        return Octree(parentState,new_max_dist)

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



