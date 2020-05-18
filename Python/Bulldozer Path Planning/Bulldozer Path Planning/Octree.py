class Octree:
    def __init__(self,state,max_distance,max_size=8):
        self._centreState = state
        self._max_distance = max_distance
        self._max_size = max_size
        self._has_children = False
        self._children = [] #list of Octrees
        self._vehicle_states = [] #list of vehicles
        self._num_states = 0


    def addState(self,state):
        if self._has_children:
            self.addStateToChild(state)
        elif len(self._vehicle_states) > self._max_size:
            self.extend()
            for vs in self._vehicle_states:
                self.addStateToChild(vs)
            self.addStateToChild(state)
            self._vehicle_states = []
        else:
            self._vehicle_states.append(state)
        self._num_states += 1



