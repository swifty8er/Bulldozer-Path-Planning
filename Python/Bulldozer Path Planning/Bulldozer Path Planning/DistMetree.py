import math
from Vehicle import Vehicle
class DistMetree:
    def __init__(self,state : Vehicle ,parent ,max_dist_metric,delta_angle,level,max_size=8,max_depth=10):
        self._centreState = state
        self._max_distance_metric = max_dist_metric
        self._centre_angle = state.theta
        self._delta_angle = delta_angle #initially 180
        self._max_angular_dist = 1.0 - math.cos(math.radians(self._delta_angle))
        self._max_euclidean_dist = self._max_distance_metric - self._max_angular_dist
        self._max_size = max_size
        self._max_depth = max_depth
        self._has_children = False
        self._children : DistMetree = [] #list of DistMetrees
        self._vehicle_states : Vehicle = [] #list of vehicles
        self._num_states = 0
        self._parent = parent
        self._level = level


    @property 
    def has_children(self):
        return self._has_children

    @property
    def num_states(self):
        return self._num_states
    
    @property
    def centreState(self):
        return self._centreState

    @property
    def vehicle_states(self):
        return self._vehicle_states


    def nearestNeighbour(self,queryState):
        if not self._has_children:
            shortest_dist = math.inf
            nn = None
            for vs in self._vehicle_states:
                dist = vs.DistanceMetric(queryState)
                if dist < shortest_dist:
                    shortest_dist = dist
                    nn = vs
            if nn == None:
                raise Exception("Error in distmetree nn function")
            return nn
        else:
            return self._children[self.closestChildWithStatesOrChildren(queryState)].nearestNeighbour(queryState)

    def stateWithinRadiusOfQuery(self,queryState,radius):
        if not self._has_children:
            for vs in self._vehicle_states:
                if vs.DistanceMetric(queryState) < radius:
                    return True
        for child in self._children:
            if child.overlapsSearchBall(queryState,radius):
                result = child.stateWithinRadiusOfQuery(queryState,radius)
                if result:
                    return True
        return False


    def overlapsSearchBall(self,queryState,radius):
        q_prime = Vehicle(abs(queryState.x-self.centreState.x),abs(queryState.y-self.centreState.y),(abs(queryState.theta)-self.centreState.theta)%360)
        new_vehicle_1 = Vehicle(self._max_euclidean_dist,self._max_euclidean_dist,(self.centreState.theta+self._delta_angle)%360)
        new_vehicle_2 = Vehicle(self._max_euclidean_dist,self._max_euclidean_dist,(self.centreState.theta-self._delta_angle)%360)
        if q_prime.x < self._max_euclidean_dist:
            return True
        if q_prime.y < self._max_euclidean_dist:
            return True
        if q_prime.theta <= self._delta_angle:
            return True
        if q_prime.DistanceMetric(new_vehicle_1) < radius:
            return True
        if q_prime.DistanceMetric(new_vehicle_2) < radius:
            return True
        return False


    def extend(self):
        self._has_children = True
        self._children = list([self.generateChildTree(index) for index in range(self._max_size)])

    def closestChildWithStatesOrChildren(self,state:Vehicle):
        shortest_dist = math.inf
        closestChildIndex = -1
        i = 0
        for child in self._children:
            if child.has_children or len(child.vehicle_states) > 0:
                dist = state.DistanceMetric(child.centreState)
                if dist < shortest_dist:
                    shortest_dist = dist
                    closestChildIndex = i
            i+=1
        if i == -1:
            raise Exception("Error in dist metree find closest child with states or children")
        return closestChildIndex
    
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
        parentState = self.getParentState(index)
        new_delta_angle = self._delta_angle / 2.0
        new_max_dist_metric = (self._max_euclidean_dist / 2.0) + (1 - math.cos(math.radians(new_delta_angle)))
        return DistMetree(parentState,self,new_max_dist_metric,new_delta_angle,self._level+1)
   

    def getParentState(self,i):
        half_dist = self._max_euclidean_dist/2.0
        half_delta_angle = self._delta_angle / 2.0
        new_angle_1 = (self._centre_angle - self._delta_angle)%360
        new_angle_2 = (self._centre_angle + self._delta_angle)%360
        if i == 0:
            return Vehicle(self.centreState.x+half_dist,self.centreState.y+half_dist,new_angle_1)
        elif i == 1:
            return Vehicle(self.centreState.x+half_dist,self.centreState.y+half_dist,new_angle_2)
        elif i == 2:
            return Vehicle(self.centreState.x+half_dist,self.centreState.y-half_dist,new_angle_1)
        elif i == 3:
            return Vehicle(self.centreState.x+half_dist,self.centreState.y-half_dist,new_angle_2)
        elif i == 4:
            return Vehicle(self.centreState.x-half_dist,self.centreState.y+half_dist,new_angle_1)
        elif i == 5:
            return Vehicle(self.centreState.x-half_dist,self.centreState.y+half_dist,new_angle_2)
        elif i == 6:
            return Vehicle(self.centreState.x-half_dist,self.centreState.y-half_dist,new_angle_1)
        elif i == 7:
            return Vehicle(self.centreState.x-half_dist,self.centreState.y-half_dist,new_angle_2)
        else:
            raise Exception("Invalid index %d passed to get parent state" % (i))

    def addState(self,state : Vehicle):
        if self._has_children:
            self.addStateToChild(state)
        elif len(self._vehicle_states) >= self._max_size and self._level < self._max_depth:
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
