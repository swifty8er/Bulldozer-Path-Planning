import math
import queue
import numpy as np
from BasicGeometry import BasicGeometry
from Pushing import Pushing
from TranspositionTable import TranspositionTable

NUM_OF_BITS = 32
TRANS_TABLE_SIZE = 15
NUM_NODES = 5000

class PQState:
    def __init__(self,map,vehicle_pose,disk_positions,vehicle_path,disk_paths,disk_being_pushed,rrt,g):
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

    @property
    def vehicle_pose(self):
        return self._vehicle_pose

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
                angle_between_heading_and_goal = abs(math.radians(self._vehicle_pose.theta) - BasicGeometry.vector_angle(BasicGeometry.vec_from_points(curr_disk_position,goal_pos)))
                dist = BasicGeometry.GoalDistanceMetric(curr_disk_position[0],curr_disk_position[1],angle_between_heading_and_goal,goal_pos[0],goal_pos[1])
                if dist < shortestDist:
                    shortestDist = dist
                    closestGoal = goal_pos
                    found = True

            i+=1

        if not found:
            raise Exception("Unable to find closest goal")
        return closestGoal

    # Use the RRT to find a path between the current position of the vehicle and the push_point
    # Use A* search
    def navigateToPushPoint(self,push_point):
        pq = queue.PriorityQueue()
        transTable = TranspositionTable(NUM_NODES*1.1,NUM_OF_BITS,TRANS_TABLE_SIZE)
        starting_state = (self._vehicle_pose.EuclideanDistance(push_point),self._vehicle_pose,[],0)
        pq.put(starting_state)
        while not pq.empty():
            curr_state = pq.get()
            (f,pose,path,g) = curr_state
            if pose == push_point:
                return (pose,path,g)
            if (transTable.isVisited(curr_state,True) == False):
                path.append(pose)
                for next_pose in self._RRT[pose]:
                    if not self._RRT.edgeCollidesWithDirtPile(pose,next_pose,self._RRT[pose][next_pose],self._disk_positions):
                        new_state = (g+next_pose.EuclideanDistance(push_point),next_pose,path,g+pose.EuclideanDistance(next_pose))
                        status = transTable.addToTable(new_state)
                        if status == "E" or status == "R":
                            pq.put(new_state)

        return (False,False,False)


    def getResultingStates(self):
        resultingStates = []
        # first consider pushing the current disk forward
        if self._disk_being_pushed != -1:
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            push_point = self._vehicle_pose
            closest_goal = self.getClosestGoalToPushLine()
            (new_disk_pos,new_vehicle_pose) = Pushing.pushDisk(push_point,curr_disk_pos,closest_goal)
            if not (curr_disk_pos[0] == new_disk_pos[0] and curr_disk_pos[1] == new_disk_pos[1]):
                new_disk_positions = np.copy(self._disk_positions)
                new_disk_positions[self._disk_being_pushed] = new_disk_pos
                new_vehicle_path = self._vehicle_path.copy()
                new_vehicle_path.append(push_point)
                new_disk_paths = self._disk_paths.copy()
                new_disk_paths[self._disk_being_pushed].append(curr_disk_pos)
                newState = PQState(self._map,new_vehicle_pose,new_disk_positions,new_vehicle_path,new_vehicle_paths,self._disk_being_pushed,self._RRT,self._g+BasicGeometry.ptDist((push_point.x,push_point.y),(new_vehicle_pose.x,new_vehicle_pose.y)))
                resultingStates.append(newState)
        # next consider navigating to a different push point on the current disk
        curr_disk_pos = self._disk_positions[self._disk_being_pushed]
        new_push_points = Pushing.getPushPoints(curr_disk_pos,self._vehicle_pose.theta)
        for push_point in new_push_points:
            if self._RRT.connectPushPoint(push_point):
                (new_vehicle_pose,new_vehicle_path,gValue) = self.navigateToPushPoint(push_point)
                if not (new_vehicle_pose == False and new_vehicle_path == False and gValue == False):
                    newState = PQState(self._map,new_vehicle_pose,self._disk_positions,new_vehicle_path,self._disk_paths,self._disk_being_pushed,self._RRT,self._g+gValue)
                    resultingStates.append(newState)
        # finally consider navigating to the push points of all other disks
        for i in range(len(self._disk_positions)):
            if i == self._disk_being_pushed:
                continue
            curr_disk_pos = self._disk_positions[i]
            new_push_points = Pushing.getPushPoints(curr_disk_pos)
            for push_point in new_push_points:
                if self._RRT.connectPushPoint:
                    (new_vehicle_pose,new_vehicle_path,gValue) = self.navigateToPushPoint(push_point)
                    if not (new_vehicle_pose == False and new_vehicle_path == False and gValue == False):
                        newState = PQState(self._map,new_vehicle_pose,self._disk_positions,new_vehicle_path,self._disk_paths,i,self._RRT,self._g+gValue)
                        resultingStates.append(newState)

        return resultingStates


