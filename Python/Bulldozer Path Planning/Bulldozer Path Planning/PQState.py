import math
import queue
import bezier
import numpy as np
from BasicGeometry import BasicGeometry
from Pushing import Pushing
from TranspositionTable import TranspositionTable
from matplotlib import pyplot as plt


NUM_OF_BITS = 32
TRANS_TABLE_SIZE = 15
NUM_NODES = 5000

class PQState:
    def __init__(self,map,vehicle_pose,disk_positions,vehicle_path,disk_paths,reached_goals,disk_being_pushed,rrt,g):
        self._map = map
        self._vehicle_pose = vehicle_pose
        self._disk_positions = disk_positions
        self._vehicle_path = vehicle_path
        self._disk_paths = disk_paths
        self._reached_goals = reached_goals
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

    @property
    def disk_positions(self):
        return self._disk_positions

    def __lt__(self,other):
        return self.f < other.f


    def plotState(self,ax,line_width=2):
        self._map.plotMapBoundaryObstacles(ax)
        for disk_pos in self._disk_positions:
            disk_circle = BasicGeometry.circlePoints(disk_pos, self._map.disk_radius, 25)
            ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)
        self.drawVehiclePose(ax)


    # a hacky zobrist hash - does not use random numbers
    def __hash__(self):
        print("Getting hash for PQ state with vehicle pos (%.2f,%.2f,%.2f) and disk positions = " % (self._vehicle_pose.x,self._vehicle_pose.y,self._vehicle_pose.theta))
        h = 0
        h = h^self.vehicle_pose.__hash__()
        for pos in self.disk_positions:
            print(pos)
            t = (pos[0],pos[1])
            h = h^hash(t)
        print("Hash = ",h)
        return h

    def isFinishState(self):
        for r in self._reached_goals:
            if not r:
                return False
        return True


    def calculateHeuristicValue(self):
        reached = self._reached_goals.copy()
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
            if not self._reached_goals[i]:
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
    # Use A* search in reverse from the push point to the current position
    def navigateToPushPoint(self,push_point,cachedPaths,axis):
        pq = queue.PriorityQueue()
        visitedNodes = {}
        starting_state = (push_point.EuclideanDistance(self._vehicle_pose),push_point,[],0)
        pq.put(starting_state)
        while not pq.empty():
            curr_state = pq.get()
            (f,pose,path,g) = curr_state
            if pose == self._vehicle_pose:
                path.append(pose)
                path.reverse()
                self.drawPath(path,axis)
                return (push_point,path,g)
            remainingPath = self.getSavedPath(pose,cachedPaths)
            if remainingPath != False:
                path.append(pose)
                path.reverse()
                new_path  = remainingPath + path
                self.drawPath(new_path,axis)
                return (push_point,new_path,g+self.calcPathLength(remainingPath))


            if pose not in visitedNodes:
                visitedNodes[pose] = True
                new_path = path.copy()
                new_path.append(pose)
                for next_pose in self._RRT.tree[pose]:
                    if self._RRT.tree[pose][next_pose] != False:
                        if not self._RRT.edgeCollidesWithDirtPile(pose,next_pose,self._RRT.tree[pose][next_pose],self._disk_positions) and not next_pose in visitedNodes:
                            new_state = (next_pose.EuclideanDistance(self._vehicle_pose),next_pose,new_path,g+self.getEdgeLength(pose,next_pose)) #change this to use the arc path length
                            pq.put(new_state)
                          

        return (False,False,False)


    def getEdgeLength(self,n1,n2):
        edge = self._RRT.tree[n1][n2]
        if edge != False:
            if isinstance(edge,bezier.curve.Curve):
                return edge.length
            else:
                try:
                    (radius,deltaTheta,direction) = edge
                except:
                    raise Exception("Invalid RRT edge found")
                if direction == "F" or direction =="R":
                    return radius
                else:
                    return radius * math.radians(deltaTheta)
        else:
            raise Exception("Empty edge found in length function")

    def calcPathLength(self,path):
        length = 0
        for i in range(len(path)-1):
            curr_node = path[i]
            next_node = path[i+1]
            length += self.getEdgeLength(curr_node,next_node)
        return length

    def getSavedPath(self,pose,cachedPaths):
        for path in cachedPaths:
            for i in range(len(path)):
                curr_pose = path[i]
                if curr_pose == pose:
                    return path[:i]
        return False

    def drawPath(self,path,ax):
        for i in range(len(path)-1):
            curr_node = path[i]
            next_node = path[i+1]
            self._RRT.drawEdge(curr_node,next_node,ax,'k-')

        plt.draw()
        plt.pause(1)
        plt.show(block=False)

    def drawVehiclePose(self,axis):
        vehicle_pos = (self.vehicle_pose.x,self.vehicle_pose.y)
        pos_circle = BasicGeometry.circlePoints(vehicle_pos, self._map.disk_radius, 25)
        axis.plot(pos_circle[0],pos_circle[1],color='red', linewidth=2)
        r = 0.5
        dx = r*math.cos(math.radians(self.vehicle_pose.theta))
        dy = r*math.sin(math.radians(self.vehicle_pose.theta))
        axis.arrow(self.vehicle_pose.x,self.vehicle_pose.y,dx,dy,width=0.05)
        plt.draw()
        plt.pause(1)
        plt.show(block=False)


    def determineGoalsReached(self,disk_positions):
        reached = [False]*len(self._map.goal_pos_xy)
        i = 0
        for goal_pos in self._map.goal_pos_xy:
            for disk_pos in disk_positions:
                if (BasicGeometry.ptDist(disk_pos,goal_pos)) < 0.05:
                    reached[i] = True
                    break
            i+=1
        return reached

    def getStateAfterPush(self,push_point,curr_disk_pos,vehicle_path,disk_being_pushed,gValue):
        closest_goal = self.getClosestGoalToPushLine(curr_disk_pos)
        (new_disk_pos,new_vehicle_pose) = Pushing.pushDisk(push_point,curr_disk_pos,closest_goal,self._map)
        if not (curr_disk_pos[0] == new_disk_pos[0] and curr_disk_pos[1] == new_disk_pos[1]):
            distance = push_point.EuclideanDistance(new_vehicle_pose)
            new_edge = (distance,0,"F")
            inv_edge = (distance,0,"R")
            self._RRT.addEdge(new_vehicle_pose,push_point,new_edge,inv_edge)
            print("Pushing disk from [%.2f,%.2f] to goal [%.2f,%.2f] results in new disk pos [%.2f,%.2f] and vehicle pose = (%.2f,%.2f,%.2f)" % (curr_disk_pos[0],curr_disk_pos[1],closest_goal[0],closest_goal[1],new_disk_pos[0],new_disk_pos[1],new_vehicle_pose.x,new_vehicle_pose.y,new_vehicle_pose.theta))
            new_disk_positions = np.copy(self._disk_positions)
            new_disk_positions[self._disk_being_pushed] = new_disk_pos
            new_vehicle_path = vehicle_path.copy()
            new_vehicle_path.append(push_point)
            new_disk_paths = self._disk_paths.copy()
            new_disk_paths[disk_being_pushed].append(curr_disk_pos)
            new_reached_goals = self.determineGoalsReached(new_disk_positions)
            #use greedy search for now i.e g = 0 f = h
            return PQState(self._map,new_vehicle_pose,new_disk_positions,new_vehicle_path,new_disk_paths,new_reached_goals,disk_being_pushed,self._RRT,0)
           
        return False

    def getResultingStates(self,axis):
        print("Getting resulting states")
        resultingStates = []
        cachedPaths = []
        # first consider pushing the current disk forward
        if self._disk_being_pushed != -1:
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            print("Current disk being pushed pos [%.2f,%.2f]" % (curr_disk_pos[0],curr_disk_pos[1]))
            push_point = self._vehicle_pose
            pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._vehicle_path,self._disk_being_pushed,self._g)
            if pushedState != False:
                resultingStates.append(pushedState)
            # next consider navigating to a different push point on the current disk
            curr_disk_pos = self._disk_positions[self._disk_being_pushed]
            new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius,self._vehicle_pose.theta)
            for push_point in new_push_points:
                print("Push point = (%.2f,%.2f,%.2f)" % (push_point.x,push_point.y,push_point.theta))
                if self._RRT.connectPushPoint(push_point):
                    (new_vehicle_pose,new_vehicle_path,gValue) = self.navigateToPushPoint(push_point,cachedPaths,axis)
                    if not (new_vehicle_pose == False and new_vehicle_path == False and gValue == False):
                        pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._vehicle_path+new_vehicle_path,self._disk_being_pushed,self._g+gValue)
                        if pushedState != False:
                            resultingStates.append(pushedState)
                            print("Added state that pushed from new push point on same disk")
                        cachedPaths.append(new_vehicle_path)
                else:
                    print("Could not connect to push point")
        # finally consider navigating to the push points of all other disks
        for i in range(len(self._disk_positions)):
            if i == self._disk_being_pushed:
                continue
            curr_disk_pos = self._disk_positions[i]
            new_push_points = Pushing.getPushPoints(curr_disk_pos,self._map.disk_radius)
            for push_point in new_push_points:
                print("Push point = (%.2f,%.2f,%.2f)" % (push_point.x,push_point.y,push_point.theta))
                if self._RRT.connectPushPoint(push_point):
                    (new_vehicle_pose,new_vehicle_path,gValue) = self.navigateToPushPoint(push_point,cachedPaths,axis)
                    if not (new_vehicle_pose == False and new_vehicle_path == False and gValue == False):
                        pushedState = self.getStateAfterPush(push_point,curr_disk_pos,self._vehicle_path+new_vehicle_path,i,self._g+gValue)
                        if pushedState != False:
                            resultingStates.append(pushedState)
                            print("Added state that pushed from new disk push point")
                        
                        cachedPaths.append(new_vehicle_path)
                else:
                    print("Could not connect to push point")
        return resultingStates


