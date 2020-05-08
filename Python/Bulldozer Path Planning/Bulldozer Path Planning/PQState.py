import math
from BasicGeometry import BasicGeometry
class PQState:
    def __init__(self,map,vehicle_pose,disk_positions,vehicle_path,disk_paths,disk_being_pushed,rrt,f,g):
        self._map = map
        self._vehicle_pose = vehicle_pose
        self._disk_positions = disk_positions
        self._vehicle_path = vehicle_path
        self._disk_paths = disk_paths
        self._RRT = rrt
        self._disk_being_pushed = disk_being_pushed #index of disk being pushed -1 = no disk
        self._g = g
        self._f = self._g + self.calculateHeuristicValue()

    
    @property
    def f(self):
        return self._f

    def __lt__(self,other):
        return self.f < other.f


    def isFinishState(self):
        sorted_disk_poses = sorted(self._disk_positions)
        sorted_goal_poses = sorted(self._map.goal_pos_xy)
        for i in range(len(sorted_disk_poses)):
            if (sorted_disk_poses[i] != sorted_goal_poses[i]):
                    return False

        return True

    def calculateHeuristicValue(self):
        reached = [False]*len(self._map.goal_pos_xy)
        h = 0
        for disk in self._disk_positions:
            closestGoal = self.getClosestGoalManhattan(disk,reached)
            index = self._map.goal_pos_xy.index(closestGoal)
            reached[index] = True
            h += BasicGeometry.manhattanDistance(disk,closestGoal)

        return h

    def getClosestGoalManhattan(self,disk_pos,reached):
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

    def getClosestGoalToPushLine(self,curr_disk_position):
        shortestDist = math.inf
        closestGoal = None
        i = 0
        found = False
        for goal_pos in self._map.goal_pos_xy:
            if not reached[i]:
                dist = BasicGeometry.GoalDistanceMetric(curr_disk_position[0],curr_disk_position[1],self._vehicle_pose.theta,goal_pos[0],goal_pos[1])
                if dist < shortestDist:
                    shortestDist = dist
                    closestGoal = goal_pos
                    found = True

            i+=1

        if not found:
            raise Exception("Unable to find closest goal")
        return closestGoal


    def getResultingStates(self):
        # first consider pushing the current disk forward
        if self._disk_being_pushed != -1:
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            push_point = self._vehicle_pose
            closest_goal = self.getClosestGoalToPushLine()
        # next consider navigating to a different push point on the current disk

        # finally consider navigating to the push points of all other disks



