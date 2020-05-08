import math
from BasicGeometry import BasicGeometry
class PQState:
    def __init__(self,map,vehicle_pose,disk_positions,vehicle_path,disk_paths,rrt,f,g):
        self._map = map
        self._vehicle_pose = vehicle_pose
        self._disk_positions = disk_positions
        self._vehicle_path = vehicle_path
        self_disk_paths = disk_paths
        self_RRT = rrt
        self._g = g
        self._f = self._g + self.calculateHeuristicValue()

    
    @property
    def f(self):
        return self._f

    def __lt__(self,other):
        return self.f < other.f

    def calculateHeuristicValue(self):
        reached = [False]*len(self._map.goal_pos_xy)
        h = 0
        for disk in self._disk_positions:
            closestGoal = self.getClosestGoal(disk,reached)
            index = self._map.goal_pos_xy.index(closestGoal)
            reached[index] = True
            h += BasicGeometry.manhattanDistance(disk,closestGoal)

        return h

    def getClosestGoal(self,disk_pos,reached):
        shortestDist = math.inf
        closestGoal = None
        i = 0
        found = False
        for goal_pos in self._map.goal_pos_xy:
            if not reached[i]:
                dist = BasicGeometry.manhattanDistance(disk_pos,goal_pos)
                if dist < shortestDist:
                    shortestDist = dist
                    closestGoal = goal_pos
                    found = True
            i+=1

        if not found:
            raise Exception("Unable to find closest goal")
        
        return closestGoal




